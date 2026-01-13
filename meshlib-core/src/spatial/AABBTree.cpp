/**
 * @file AABBTree.cpp
 * @brief Implementation of AABB trees for spatial queries
 */

#include "meshlib/spatial/AABBTree.h"
#include "meshlib/mesh/Mesh.h"
#include "meshlib/mesh/PointCloud.h"
#include <algorithm>
#include <queue>
#include <numeric>
#include <cmath>

namespace meshlib {

// ==================== Mesh AABB Tree ====================

AABBTree::AABBTree(const Mesh& mesh) : mesh_(&mesh) {
    buildTree(mesh);
}

void AABBTree::buildTree(const Mesh& mesh) {
    const auto& vertices = mesh.vertices();
    const auto& triangles = mesh.triangles();
    
    if (triangles.empty()) return;
    
    // Compute bounding boxes and centers for all triangles
    std::vector<Box3f> boxes(triangles.size());
    std::vector<Vector3f> centers(triangles.size());
    std::vector<FaceId> faces(triangles.size());
    
    for (size_t i = 0; i < triangles.size(); ++i) {
        const auto& tri = triangles[i];
        const Vector3f& v0 = vertices[tri[0]];
        const Vector3f& v1 = vertices[tri[1]];
        const Vector3f& v2 = vertices[tri[2]];
        
        boxes[i] = Box3f{};
        boxes[i].include(v0);
        boxes[i].include(v1);
        boxes[i].include(v2);
        
        centers[i] = (v0 + v1 + v2) * (1.0f / 3.0f);
        faces[i] = FaceId{static_cast<int>(i)};
    }
    
    // Allocate nodes (2n - 1 nodes for n leaves)
    nodes_.resize(triangles.size() * 2 - 1);
    
    // Build tree recursively
    buildSubtree(rootNodeId(), faces, 0, faces.size(), boxes, centers);
}

void AABBTree::buildSubtree(
    NodeId nodeId,
    std::vector<FaceId>& faces,
    size_t begin, size_t end,
    const std::vector<Box3f>& boxes,
    const std::vector<Vector3f>& centers) {
    
    Node& node = nodes_[nodeId];
    
    // Compute bounding box for this node
    node.box = Box3f{};
    for (size_t i = begin; i < end; ++i) {
        node.box.include(boxes[faces[i]]);
    }
    
    if (end - begin == 1) {
        // Leaf node
        node.setLeafId(faces[begin]);
        return;
    }
    
    // Find split axis (longest extent)
    Vector3f extent = node.box.size();
    int axis = 0;
    if (extent.y > extent.x) axis = 1;
    if (extent.z > extent[axis]) axis = 2;
    
    // Sort by center along split axis
    size_t mid = (begin + end) / 2;
    std::nth_element(
        faces.begin() + begin,
        faces.begin() + mid,
        faces.begin() + end,
        [&centers, axis](FaceId a, FaceId b) {
            return centers[a][axis] < centers[b][axis];
        });
    
    // Create child nodes
    node.l = NodeId{static_cast<int>(nodeId) + 1};
    node.r = NodeId{static_cast<int>(nodeId) + static_cast<int>(2 * (mid - begin))};
    
    // Recursively build children
    buildSubtree(node.l, faces, begin, mid, boxes, centers);
    buildSubtree(node.r, faces, mid, end, boxes, centers);
}

AABBTree::RayIntersection AABBTree::findRayIntersection(
    const Vector3f& origin,
    const Vector3f& direction,
    float maxDistance) const {
    
    RayIntersection result;
    if (nodes_.empty() || !mesh_) return result;
    
    const auto& vertices = mesh_->vertices();
    const auto& triangles = mesh_->triangles();
    
    // Inverse direction for fast box tests
    Vector3f invDir{
        direction.x != 0 ? 1.0f / direction.x : std::numeric_limits<float>::max(),
        direction.y != 0 ? 1.0f / direction.y : std::numeric_limits<float>::max(),
        direction.z != 0 ? 1.0f / direction.z : std::numeric_limits<float>::max()
    };
    
    float bestT = maxDistance;
    
    // Stack-based traversal
    std::vector<NodeId> stack;
    stack.reserve(64);
    stack.push_back(rootNodeId());
    
    while (!stack.empty()) {
        NodeId nid = stack.back();
        stack.pop_back();
        
        const Node& node = nodes_[nid];
        
        // Test ray-box intersection
        float tmin = 0, tmax = bestT;
        for (int i = 0; i < 3; ++i) {
            float t1 = (node.box.min[i] - origin[i]) * invDir[i];
            float t2 = (node.box.max[i] - origin[i]) * invDir[i];
            if (t1 > t2) std::swap(t1, t2);
            tmin = std::max(tmin, t1);
            tmax = std::min(tmax, t2);
        }
        
        if (tmin > tmax) continue;  // No intersection with box
        
        if (node.leaf()) {
            // Test ray-triangle intersection (Möller–Trumbore)
            FaceId faceId = node.leafId();
            const auto& tri = triangles[faceId];
            const Vector3f& v0 = vertices[tri[0]];
            const Vector3f& v1 = vertices[tri[1]];
            const Vector3f& v2 = vertices[tri[2]];
            
            Vector3f edge1 = v1 - v0;
            Vector3f edge2 = v2 - v0;
            Vector3f h = cross(direction, edge2);
            float a = dot(edge1, h);
            
            if (std::abs(a) < 1e-10f) continue;
            
            float f = 1.0f / a;
            Vector3f s = origin - v0;
            float u = f * dot(s, h);
            
            if (u < 0.0f || u > 1.0f) continue;
            
            Vector3f q = cross(s, edge1);
            float v = f * dot(direction, q);
            
            if (v < 0.0f || u + v > 1.0f) continue;
            
            float t = f * dot(edge2, q);
            
            if (t > 1e-6f && t < bestT) {
                bestT = t;
                result.faceId = faceId;
                result.t = t;
                result.point = origin + direction * t;
                result.barycentric = Vector2f{u, v};
            }
        } else {
            // Push children (closer one last so it's processed first)
            float tL = (nodes_[node.l].box.center() - origin).dot(direction);
            float tR = (nodes_[node.r].box.center() - origin).dot(direction);
            
            if (tL < tR) {
                stack.push_back(node.r);
                stack.push_back(node.l);
            } else {
                stack.push_back(node.l);
                stack.push_back(node.r);
            }
        }
    }
    
    return result;
}

bool AABBTree::rayHits(
    const Vector3f& origin,
    const Vector3f& direction,
    float maxDistance) const {
    
    return findRayIntersection(origin, direction, maxDistance).valid();
}

AABBTree::ClosestPoint AABBTree::findClosestPoint(
    const Vector3f& point,
    float maxDistSq) const {
    
    ClosestPoint result;
    if (nodes_.empty() || !mesh_) return result;
    
    const auto& vertices = mesh_->vertices();
    const auto& triangles = mesh_->triangles();
    
    float bestDistSq = maxDistSq;
    
    // Priority queue for traversal (distance, nodeId)
    using QueueItem = std::pair<float, NodeId>;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> pq;
    pq.push({0.0f, rootNodeId()});
    
    while (!pq.empty()) {
        auto [dist, nid] = pq.top();
        pq.pop();
        
        if (dist > bestDistSq) break;
        
        const Node& node = nodes_[nid];
        
        if (node.leaf()) {
            // Find closest point on triangle
            FaceId faceId = node.leafId();
            const auto& tri = triangles[faceId];
            const Vector3f& v0 = vertices[tri[0]];
            const Vector3f& v1 = vertices[tri[1]];
            const Vector3f& v2 = vertices[tri[2]];
            
            // Project point onto triangle plane
            Vector3f edge0 = v1 - v0;
            Vector3f edge1 = v2 - v0;
            Vector3f v0p = point - v0;
            
            float d00 = dot(edge0, edge0);
            float d01 = dot(edge0, edge1);
            float d11 = dot(edge1, edge1);
            float d20 = dot(v0p, edge0);
            float d21 = dot(v0p, edge1);
            
            float denom = d00 * d11 - d01 * d01;
            float u = (d11 * d20 - d01 * d21) / denom;
            float v = (d00 * d21 - d01 * d20) / denom;
            
            // Clamp to triangle
            if (u < 0) { u = 0; v = std::clamp(v, 0.0f, 1.0f); }
            else if (v < 0) { v = 0; u = std::clamp(u, 0.0f, 1.0f); }
            else if (u + v > 1) { 
                float t = (u + v);
                u /= t; v /= t;
            }
            
            Vector3f closest = v0 + edge0 * u + edge1 * v;
            float distSq = (closest - point).lengthSq();
            
            if (distSq < bestDistSq) {
                bestDistSq = distSq;
                result.faceId = faceId;
                result.point = closest;
                result.distSq = distSq;
                result.barycentric = Vector2f{u, v};
            }
        } else {
            // Add children with their minimum distances
            float distL = nodes_[node.l].box.signedDistance(point);
            float distR = nodes_[node.r].box.signedDistance(point);
            distL = distL * distL;
            distR = distR * distR;
            
            if (distL < bestDistSq) pq.push({distL, node.l});
            if (distR < bestDistSq) pq.push({distR, node.r});
        }
    }
    
    return result;
}

FaceBitSet AABBTree::findTrianglesInBox(const Box3f& box) const {
    FaceBitSet result;
    if (nodes_.empty() || !mesh_) return result;
    
    result.resize(mesh_->triangleCount());
    
    std::vector<NodeId> stack;
    stack.push_back(rootNodeId());
    
    while (!stack.empty()) {
        NodeId nid = stack.back();
        stack.pop_back();
        
        const Node& node = nodes_[nid];
        
        if (!node.box.intersects(box)) continue;
        
        if (node.leaf()) {
            result.set(node.leafId());
        } else {
            stack.push_back(node.l);
            stack.push_back(node.r);
        }
    }
    
    return result;
}

FaceBitSet AABBTree::findTrianglesInBall(const Vector3f& center, float radius) const {
    FaceBitSet result;
    if (nodes_.empty() || !mesh_) return result;
    
    result.resize(mesh_->triangleCount());
    float radiusSq = radius * radius;
    
    std::vector<NodeId> stack;
    stack.push_back(rootNodeId());
    
    while (!stack.empty()) {
        NodeId nid = stack.back();
        stack.pop_back();
        
        const Node& node = nodes_[nid];
        
        // Check if ball intersects box
        float distSq = node.box.signedDistance(center);
        distSq = distSq * distSq;
        if (distSq > radiusSq) continue;
        
        if (node.leaf()) {
            result.set(node.leafId());
        } else {
            stack.push_back(node.l);
            stack.push_back(node.r);
        }
    }
    
    return result;
}

// ==================== Point Cloud AABB Tree ====================

AABBTreePoints::AABBTreePoints(const PointCloud& points) {
    ownedPoints_ = points.points();
    points_ = &ownedPoints_;
    numPoints_ = ownedPoints_.size();
    buildTree(ownedPoints_);
}

AABBTreePoints::AABBTreePoints(const std::vector<Vector3f>& points) 
    : points_(&points), numPoints_(points.size()) {
    buildTree(points);
}

void AABBTreePoints::buildTree(const std::vector<Vector3f>& points) {
    if (points.empty()) return;
    
    // Create point indices
    std::vector<VertId> pointIds(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        pointIds[i] = VertId{static_cast<int>(i)};
    }
    
    // Allocate nodes
    nodes_.resize(points.size() * 2 - 1);
    
    // Build tree
    buildSubtree(NodeId{0}, pointIds, 0, pointIds.size(), points);
}

void AABBTreePoints::buildSubtree(
    NodeId nodeId,
    std::vector<VertId>& pointIds,
    size_t begin, size_t end,
    const std::vector<Vector3f>& points) {
    
    Node& node = nodes_[nodeId];
    
    // Compute bounding box
    node.box = Box3f{};
    for (size_t i = begin; i < end; ++i) {
        node.box.include(points[pointIds[i]]);
    }
    
    if (end - begin == 1) {
        // Leaf node
        node.l = NodeId{static_cast<int>(pointIds[begin])};
        node.r = NodeId{};
        return;
    }
    
    // Find split axis
    Vector3f extent = node.box.size();
    int axis = 0;
    if (extent.y > extent.x) axis = 1;
    if (extent.z > extent[axis]) axis = 2;
    
    // Sort by position along axis
    size_t mid = (begin + end) / 2;
    std::nth_element(
        pointIds.begin() + begin,
        pointIds.begin() + mid,
        pointIds.begin() + end,
        [&points, axis](VertId a, VertId b) {
            return points[a][axis] < points[b][axis];
        });
    
    // Create children
    node.l = NodeId{static_cast<int>(nodeId) + 1};
    node.r = NodeId{static_cast<int>(nodeId) + static_cast<int>(2 * (mid - begin))};
    
    buildSubtree(node.l, pointIds, begin, mid, points);
    buildSubtree(node.r, pointIds, mid, end, points);
}

AABBTreePoints::ClosestPoint AABBTreePoints::findClosestPoint(
    const Vector3f& query,
    float maxDistSq) const {
    
    ClosestPoint result;
    if (nodes_.empty() || !points_) return result;
    
    float bestDistSq = maxDistSq;
    
    using QueueItem = std::pair<float, NodeId>;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> pq;
    pq.push({0.0f, NodeId{0}});
    
    while (!pq.empty()) {
        auto [dist, nid] = pq.top();
        pq.pop();
        
        if (dist > bestDistSq) break;
        
        const Node& node = nodes_[nid];
        
        if (node.leaf()) {
            VertId pid = node.pointId();
            float distSq = ((*points_)[pid] - query).lengthSq();
            if (distSq < bestDistSq) {
                bestDistSq = distSq;
                result.pointId = pid;
                result.distSq = distSq;
            }
        } else {
            float distL = nodes_[node.l].box.signedDistance(query);
            float distR = nodes_[node.r].box.signedDistance(query);
            distL = distL * distL;
            distR = distR * distR;
            
            if (distL < bestDistSq) pq.push({distL, node.l});
            if (distR < bestDistSq) pq.push({distR, node.r});
        }
    }
    
    return result;
}

std::vector<VertId> AABBTreePoints::findKNearest(const Vector3f& query, int k) const {
    std::vector<VertId> result;
    if (nodes_.empty() || !points_ || k <= 0) return result;
    
    // Max-heap of (distSq, pointId)
    using HeapItem = std::pair<float, VertId>;
    std::priority_queue<HeapItem> nearest;
    
    std::vector<NodeId> stack;
    stack.push_back(NodeId{0});
    
    float maxDistSq = std::numeric_limits<float>::max();
    
    while (!stack.empty()) {
        NodeId nid = stack.back();
        stack.pop_back();
        
        const Node& node = nodes_[nid];
        
        float boxDist = node.box.signedDistance(query);
        if (boxDist * boxDist > maxDistSq) continue;
        
        if (node.leaf()) {
            VertId pid = node.pointId();
            float distSq = ((*points_)[pid] - query).lengthSq();
            
            if (static_cast<int>(nearest.size()) < k) {
                nearest.push({distSq, pid});
                if (static_cast<int>(nearest.size()) == k) {
                    maxDistSq = nearest.top().first;
                }
            } else if (distSq < nearest.top().first) {
                nearest.pop();
                nearest.push({distSq, pid});
                maxDistSq = nearest.top().first;
            }
        } else {
            stack.push_back(node.l);
            stack.push_back(node.r);
        }
    }
    
    result.reserve(nearest.size());
    while (!nearest.empty()) {
        result.push_back(nearest.top().second);
        nearest.pop();
    }
    std::reverse(result.begin(), result.end());
    
    return result;
}

VertBitSet AABBTreePoints::findPointsInBall(const Vector3f& center, float radius) const {
    VertBitSet result;
    if (nodes_.empty() || !points_) return result;
    
    result.resize(numPoints_);
    float radiusSq = radius * radius;
    
    std::vector<NodeId> stack;
    stack.push_back(NodeId{0});
    
    while (!stack.empty()) {
        NodeId nid = stack.back();
        stack.pop_back();
        
        const Node& node = nodes_[nid];
        
        float dist = node.box.signedDistance(center);
        if (dist * dist > radiusSq) continue;
        
        if (node.leaf()) {
            VertId pid = node.pointId();
            if (((*points_)[pid] - center).lengthSq() <= radiusSq) {
                result.set(pid);
            }
        } else {
            stack.push_back(node.l);
            stack.push_back(node.r);
        }
    }
    
    return result;
}

VertBitSet AABBTreePoints::findPointsInBox(const Box3f& box) const {
    VertBitSet result;
    if (nodes_.empty() || !points_) return result;
    
    result.resize(numPoints_);
    
    std::vector<NodeId> stack;
    stack.push_back(NodeId{0});
    
    while (!stack.empty()) {
        NodeId nid = stack.back();
        stack.pop_back();
        
        const Node& node = nodes_[nid];
        
        if (!node.box.intersects(box)) continue;
        
        if (node.leaf()) {
            VertId pid = node.pointId();
            if (box.contains((*points_)[pid])) {
                result.set(pid);
            }
        } else {
            stack.push_back(node.l);
            stack.push_back(node.r);
        }
    }
    
    return result;
}

} // namespace meshlib
