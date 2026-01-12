/**
 * @file ICP.cpp
 * @brief Implementation of Iterative Closest Point alignment algorithm
 */

#include "meshlib/algorithms/ICP.h"
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <algorithm>
#include <random>
#include <cmath>
#include <numeric>

namespace meshlib {

namespace {

// Simple KD-tree for nearest neighbor search (axis-aligned bounding box hierarchy)
class KDTree {
public:
    explicit KDTree(const std::vector<Vector3f>& points) : points_(points) {
        indices_.resize(points.size());
        std::iota(indices_.begin(), indices_.end(), 0);
        
        if (!points.empty()) {
            buildTree(0, static_cast<int>(points.size()), 0);
        }
    }
    
    int findNearest(const Vector3f& query, float& outDistance) const {
        if (points_.empty()) {
            outDistance = std::numeric_limits<float>::max();
            return -1;
        }
        
        int bestIdx = -1;
        float bestDist = std::numeric_limits<float>::max();
        searchNearest(query, 0, static_cast<int>(points_.size()), 0, bestIdx, bestDist);
        outDistance = bestDist;
        return bestIdx;
    }

private:
    struct Node {
        int splitAxis;
        float splitValue;
    };
    
    std::vector<Vector3f> points_;
    std::vector<int> indices_;
    std::vector<Node> nodes_;
    
    void buildTree(int start, int end, int depth) {
        if (end - start <= 1) return;
        
        int axis = depth % 3;
        int mid = (start + end) / 2;
        
        // Partial sort to find median
        std::nth_element(
            indices_.begin() + start,
            indices_.begin() + mid,
            indices_.begin() + end,
            [this, axis](int a, int b) {
                return points_[a][axis] < points_[b][axis];
            }
        );
        
        // Recursively build children
        buildTree(start, mid, depth + 1);
        buildTree(mid + 1, end, depth + 1);
    }
    
    void searchNearest(const Vector3f& query, int start, int end, int depth,
                       int& bestIdx, float& bestDist) const {
        if (end <= start) return;
        
        // Check current segment
        for (int i = start; i < end; ++i) {
            int idx = indices_[i];
            float dist = (points_[idx] - query).lengthSq();
            if (dist < bestDist) {
                bestDist = dist;
                bestIdx = idx;
            }
        }
    }
};

// Sample points uniformly
std::vector<int> sampleUniform(size_t totalCount, size_t sampleCount, std::mt19937& rng) {
    std::vector<int> indices(totalCount);
    std::iota(indices.begin(), indices.end(), 0);
    
    if (sampleCount >= totalCount) {
        return indices;
    }
    
    std::shuffle(indices.begin(), indices.end(), rng);
    indices.resize(sampleCount);
    return indices;
}

// Extract points from mesh
std::vector<Vector3f> getPointsFromMesh(const Mesh& mesh) {
    return mesh.vertices();
}

// Extract points from point cloud
std::vector<Vector3f> getPointsFromCloud(const PointCloud& cloud) {
    return cloud.points();
}

// Compute centroid
Vector3f computeCentroid(const std::vector<Vector3f>& points, const std::vector<int>& indices) {
    Vector3f sum = Vector3f::zero();
    for (int idx : indices) {
        sum = sum + points[idx];
    }
    return sum * (1.0f / indices.size());
}

// ICP iteration using SVD
ICPResult runICP(
    const std::vector<Vector3f>& sourcePoints,
    const std::vector<Vector3f>& targetPoints,
    const ICPParams& params) {
    
    ICPResult result;
    
    if (sourcePoints.empty() || targetPoints.empty()) {
        result.status.code = ErrorCode::InvalidInput;
        result.status.message = "Empty point set";
        return result;
    }
    
    // Build KD-tree for target
    KDTree kdTree(targetPoints);
    
    // Initialize transformation
    result.transform = AffineTransform3f::identity();
    result.rotation = Matrix3f::identity();
    result.translation = Vector3f::zero();
    result.scale = 1.0f;
    
    // Current transformed source points
    std::vector<Vector3f> currentSource = sourcePoints;
    
    // Random number generator for sampling
    std::mt19937 rng(42);
    
    float prevError = std::numeric_limits<float>::max();
    
    for (int iter = 0; iter < params.maxIterations; ++iter) {
        // Sample source points
        std::vector<int> sampleIndices;
        if (params.sampleCount > 0 && params.sampleCount < currentSource.size()) {
            sampleIndices = sampleUniform(currentSource.size(), params.sampleCount, rng);
        } else {
            sampleIndices.resize(currentSource.size());
            std::iota(sampleIndices.begin(), sampleIndices.end(), 0);
        }
        
        // Find correspondences
        std::vector<std::pair<int, int>> correspondences;
        std::vector<float> distances;
        
        for (int srcIdx : sampleIndices) {
            float dist;
            int tgtIdx = kdTree.findNearest(currentSource[srcIdx], dist);
            
            if (tgtIdx >= 0) {
                float actualDist = std::sqrt(dist);
                if (actualDist < params.maxCorrespondenceDistance) {
                    correspondences.emplace_back(srcIdx, tgtIdx);
                    distances.push_back(actualDist);
                }
            }
        }
        
        if (correspondences.size() < 3) {
            result.status.code = ErrorCode::OperationFailed;
            result.status.message = "Insufficient correspondences";
            return result;
        }
        
        // Outlier rejection
        if (params.rejectOutliers && correspondences.size() > 10) {
            float meanDist = std::accumulate(distances.begin(), distances.end(), 0.0f) / distances.size();
            float variance = 0.0f;
            for (float d : distances) {
                variance += (d - meanDist) * (d - meanDist);
            }
            float stdDev = std::sqrt(variance / distances.size());
            float threshold = meanDist + params.outlierRejectionStdDev * stdDev;
            
            std::vector<std::pair<int, int>> filteredCorr;
            for (size_t i = 0; i < correspondences.size(); ++i) {
                if (distances[i] <= threshold) {
                    filteredCorr.push_back(correspondences[i]);
                }
            }
            correspondences = std::move(filteredCorr);
        }
        
        if (correspondences.size() < 3) {
            result.status.code = ErrorCode::OperationFailed;
            result.status.message = "Insufficient correspondences after outlier rejection";
            return result;
        }
        
        // Compute centroids
        Vector3f srcCentroid = Vector3f::zero();
        Vector3f tgtCentroid = Vector3f::zero();
        
        for (const auto& [srcIdx, tgtIdx] : correspondences) {
            srcCentroid = srcCentroid + currentSource[srcIdx];
            tgtCentroid = tgtCentroid + targetPoints[tgtIdx];
        }
        srcCentroid = srcCentroid * (1.0f / correspondences.size());
        tgtCentroid = tgtCentroid * (1.0f / correspondences.size());
        
        // Build covariance matrix using Eigen
        Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
        
        for (const auto& [srcIdx, tgtIdx] : correspondences) {
            Eigen::Vector3f ps(
                currentSource[srcIdx].x - srcCentroid.x,
                currentSource[srcIdx].y - srcCentroid.y,
                currentSource[srcIdx].z - srcCentroid.z
            );
            Eigen::Vector3f pt(
                targetPoints[tgtIdx].x - tgtCentroid.x,
                targetPoints[tgtIdx].y - tgtCentroid.y,
                targetPoints[tgtIdx].z - tgtCentroid.z
            );
            H += ps * pt.transpose();
        }
        
        // SVD decomposition
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3f R = svd.matrixV() * svd.matrixU().transpose();
        
        // Handle reflection case
        if (R.determinant() < 0) {
            Eigen::Matrix3f V = svd.matrixV();
            V.col(2) *= -1;
            R = V * svd.matrixU().transpose();
        }
        
        // Compute translation
        Eigen::Vector3f srcC(srcCentroid.x, srcCentroid.y, srcCentroid.z);
        Eigen::Vector3f tgtC(tgtCentroid.x, tgtCentroid.y, tgtCentroid.z);
        Eigen::Vector3f t = tgtC - R * srcC;
        
        // Apply transformation to source points
        for (auto& p : currentSource) {
            Eigen::Vector3f v(p.x, p.y, p.z);
            Eigen::Vector3f vt = R * v + t;
            p = Vector3f(vt.x(), vt.y(), vt.z());
        }
        
        // Update total transformation
        Matrix3f deltaR;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                deltaR(i, j) = R(i, j);
            }
        }
        Vector3f deltaT(t.x(), t.y(), t.z());
        
        result.rotation = deltaR * result.rotation;
        result.translation = deltaR * result.translation + deltaT;
        
        // Compute error
        float error = 0.0f;
        for (const auto& [srcIdx, tgtIdx] : correspondences) {
            Vector3f diff = currentSource[srcIdx] - targetPoints[tgtIdx];
            error += diff.lengthSq();
        }
        error /= correspondences.size();
        
        result.meanSquaredError = error;
        result.rmse = std::sqrt(error);
        result.iterations = iter + 1;
        result.correspondenceCount = correspondences.size();
        
        // Check convergence
        if (std::abs(prevError - error) < params.convergenceThreshold) {
            result.converged = true;
            break;
        }
        
        prevError = error;
        
        // Progress callback
        if (params.progressCallback) {
            float progress = static_cast<float>(iter + 1) / params.maxIterations;
            if (!params.progressCallback(progress)) {
                result.status.code = ErrorCode::OperationFailed;
                result.status.message = "Operation cancelled";
                return result;
            }
        }
    }
    
    // Build final transform
    result.transform = AffineTransform3f(result.rotation, result.translation);
    result.status.code = ErrorCode::Success;
    
    return result;
}

} // anonymous namespace

ICPResult icp(const Mesh& source, const Mesh& target, const ICPParams& params) {
    auto sourcePoints = getPointsFromMesh(source);
    auto targetPoints = getPointsFromMesh(target);
    return runICP(sourcePoints, targetPoints, params);
}

ICPResult icp(const PointCloud& source, const PointCloud& target, const ICPParams& params) {
    auto sourcePoints = getPointsFromCloud(source);
    auto targetPoints = getPointsFromCloud(target);
    return runICP(sourcePoints, targetPoints, params);
}

ICPResult icp(const PointCloud& source, const Mesh& target, const ICPParams& params) {
    auto sourcePoints = getPointsFromCloud(source);
    auto targetPoints = getPointsFromMesh(target);
    return runICP(sourcePoints, targetPoints, params);
}

ICPResult icp(const Mesh& source, const PointCloud& target, const ICPParams& params) {
    auto sourcePoints = getPointsFromMesh(source);
    auto targetPoints = getPointsFromCloud(target);
    return runICP(sourcePoints, targetPoints, params);
}

AffineTransform3f alignMeshes(const Mesh& source, const Mesh& target, int maxIterations) {
    ICPParams params;
    params.maxIterations = maxIterations;
    params.method = ICPMethod::PointToPlane;
    
    auto result = icp(source, target, params);
    return result.transform;
}

float alignmentError(const Mesh& meshA, const Mesh& meshB) {
    const auto& pointsA = meshA.vertices();
    const auto& pointsB = meshB.vertices();
    
    if (pointsA.empty() || pointsB.empty()) {
        return std::numeric_limits<float>::max();
    }
    
    KDTree kdTree(pointsB);
    
    float totalError = 0.0f;
    for (const auto& p : pointsA) {
        float dist;
        kdTree.findNearest(p, dist);
        totalError += std::sqrt(dist);
    }
    
    return totalError / pointsA.size();
}

AffineTransform3f roughAlignByBoundingBox(const Mesh& source, const Mesh& target) {
    Box3f srcBox = source.boundingBox();
    Box3f tgtBox = target.boundingBox();
    
    Vector3f srcCenter = srcBox.center();
    Vector3f tgtCenter = tgtBox.center();
    
    // Simple translation to align centers
    Vector3f translation = tgtCenter - srcCenter;
    
    // Optional: compute scale factor
    Vector3f srcSize = srcBox.size();
    Vector3f tgtSize = tgtBox.size();
    
    // Use average scale
    float srcDiag = srcSize.length();
    float tgtDiag = tgtSize.length();
    float scale = (srcDiag > 1e-6f) ? (tgtDiag / srcDiag) : 1.0f;
    
    // Create transformation
    Matrix3f scaleMat = Matrix3f::scale(scale);
    
    return AffineTransform3f(scaleMat, translation);
}

} // namespace meshlib
