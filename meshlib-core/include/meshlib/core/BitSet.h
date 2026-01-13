#pragma once

/**
 * @file BitSet.h
 * @brief Efficient bit set and typed bit set for mesh element selections
 * 
 * Industrial-strength port of MeshLib's MRBitSet.h providing:
 * - BitSet: Dynamic bit array with efficient operations
 * - TypedBitSet<I>: Type-safe bit set indexed by Id types
 * - SetBitIterator: Efficient iteration over set bits
 */

#include "meshlib/config.h"
#include "Id.h"
#include <vector>
#include <cstdint>
#include <cassert>
#include <algorithm>
#include <iterator>

namespace meshlib {

/**
 * @brief Dynamic bit array similar to std::vector<bool> but with better semantics
 * 
 * Key features:
 * - 64-bit blocks for efficient operations
 * - Bits after size() are always considered off
 * - Efficient set operations (and, or, xor, subtract)
 * - Iteration over set bits
 */
class MESHLIB_API BitSet {
public:
    using BlockType = uint64_t;
    static constexpr size_t BitsPerBlock = sizeof(BlockType) * 8;
    static constexpr size_t npos = static_cast<size_t>(-1);
    
    using size_type = size_t;
    using IndexType = size_t;
    
    // ==================== Construction ====================
    
    /// Creates empty bitset
    BitSet() noexcept = default;
    
    /// Creates bitset of given size filled with given value
    explicit BitSet(size_t numBits, bool fillValue = false) {
        resize(numBits, fillValue);
    }
    
    /// Creates bitset from given blocks
    static BitSet fromBlocks(std::vector<BlockType>&& blocks) {
        BitSet res;
        res.blocks_ = std::move(blocks);
        res.numBits_ = res.blocks_.size() * BitsPerBlock;
        return res;
    }
    
    // ==================== Size Management ====================
    
    void reserve(size_type numBits) { 
        blocks_.reserve(calcNumBlocks(numBits)); 
    }
    
    void resize(size_type numBits, bool fillValue = false) {
        size_t oldBlocks = blocks_.size();
        size_t newBlocks = calcNumBlocks(numBits);
        
        blocks_.resize(newBlocks, fillValue ? ~BlockType{0} : BlockType{0});
        numBits_ = numBits;
        
        if (fillValue && newBlocks > 0) {
            // Clear unused bits in last block
            resetUnusedBits();
        }
    }
    
    void clear() { 
        numBits_ = 0; 
        blocks_.clear(); 
    }
    
    void shrink_to_fit() { 
        blocks_.shrink_to_fit(); 
    }
    
    [[nodiscard]] bool empty() const noexcept { return numBits_ == 0; }
    [[nodiscard]] size_type size() const noexcept { return numBits_; }
    [[nodiscard]] size_type num_blocks() const noexcept { return blocks_.size(); }
    [[nodiscard]] size_type capacity() const noexcept { return blocks_.capacity() * BitsPerBlock; }
    
    // ==================== Bit Access ====================
    
    /// Test bit without bounds checking (bit must be < size())
    [[nodiscard]] bool uncheckedTest(IndexType n) const {
        assert(n < size());
        return blocks_[blockIndex(n)] & bitMask(n);
    }
    
    /// Test bit with bounds checking (bits >= size() are considered off)
    [[nodiscard]] bool test(IndexType n) const {
        return n < size() && uncheckedTest(n);
    }
    
    /// Test and set bit, returns old value
    [[nodiscard]] bool test_set(IndexType n, bool val = true) {
        if (val || n < size()) {
            return uncheckedTestSet(n, val);
        }
        return false;
    }
    
    [[nodiscard]] bool uncheckedTestSet(IndexType n, bool val = true) {
        assert(n < size());
        bool old = uncheckedTest(n);
        if (old != val) set(n, val);
        return old;
    }
    
    // ==================== Bit Modification ====================
    
    /// Set single bit
    BitSet& set(IndexType n, bool val = true) {
        if (val) {
            assert(n < size());
            blocks_[blockIndex(n)] |= bitMask(n);
        } else {
            reset(n);
        }
        return *this;
    }
    
    /// Set all bits
    BitSet& set() {
        for (auto& block : blocks_) {
            block = ~BlockType{0};
        }
        resetUnusedBits();
        return *this;
    }
    
    /// Reset single bit
    BitSet& reset(IndexType n) {
        if (n < size()) {
            blocks_[blockIndex(n)] &= ~bitMask(n);
        }
        return *this;
    }
    
    /// Reset all bits
    BitSet& reset() {
        for (auto& block : blocks_) {
            block = BlockType{0};
        }
        return *this;
    }
    
    /// Flip single bit
    BitSet& flip(IndexType n) {
        assert(n < size());
        blocks_[blockIndex(n)] ^= bitMask(n);
        return *this;
    }
    
    /// Flip all bits
    BitSet& flip() {
        for (auto& block : blocks_) {
            block = ~block;
        }
        resetUnusedBits();
        return *this;
    }
    
    /// Add a bit at the end
    void push_back(bool val) {
        auto n = numBits_++;
        if (bitIndex(n) == 0) {
            blocks_.push_back(BlockType{});
        }
        set(n, val);
    }
    
    /// Remove last bit
    void pop_back() {
        assert(numBits_ > 0);
        if (bitIndex(numBits_) == 1) {
            blocks_.pop_back();
        } else {
            reset(numBits_ - 1);
        }
        --numBits_;
    }
    
    // ==================== Set Operations ====================
    
    BitSet& operator&=(const BitSet& b) {
        size_t minBlocks = std::min(blocks_.size(), b.blocks_.size());
        for (size_t i = 0; i < minBlocks; ++i) {
            blocks_[i] &= b.blocks_[i];
        }
        // Bits beyond b's size become 0
        for (size_t i = minBlocks; i < blocks_.size(); ++i) {
            blocks_[i] = 0;
        }
        return *this;
    }
    
    BitSet& operator|=(const BitSet& b) {
        if (b.blocks_.size() > blocks_.size()) {
            blocks_.resize(b.blocks_.size(), 0);
        }
        for (size_t i = 0; i < b.blocks_.size(); ++i) {
            blocks_[i] |= b.blocks_[i];
        }
        if (b.numBits_ > numBits_) {
            numBits_ = b.numBits_;
        }
        return *this;
    }
    
    BitSet& operator^=(const BitSet& b) {
        if (b.blocks_.size() > blocks_.size()) {
            blocks_.resize(b.blocks_.size(), 0);
        }
        for (size_t i = 0; i < b.blocks_.size(); ++i) {
            blocks_[i] ^= b.blocks_[i];
        }
        if (b.numBits_ > numBits_) {
            numBits_ = b.numBits_;
        }
        return *this;
    }
    
    /// Set difference: *this = *this - b
    BitSet& operator-=(const BitSet& b) {
        size_t minBlocks = std::min(blocks_.size(), b.blocks_.size());
        for (size_t i = 0; i < minBlocks; ++i) {
            blocks_[i] &= ~b.blocks_[i];
        }
        return *this;
    }
    
    // ==================== Queries ====================
    
    /// Returns true if all bits are set
    [[nodiscard]] bool all() const {
        if (empty()) return true;
        
        // Check all complete blocks
        size_t fullBlocks = numBits_ / BitsPerBlock;
        for (size_t i = 0; i < fullBlocks; ++i) {
            if (blocks_[i] != ~BlockType{0}) return false;
        }
        
        // Check last partial block
        size_t remaining = numBits_ % BitsPerBlock;
        if (remaining > 0) {
            BlockType mask = (BlockType{1} << remaining) - 1;
            if ((blocks_.back() & mask) != mask) return false;
        }
        
        return true;
    }
    
    /// Returns true if any bit is set
    [[nodiscard]] bool any() const {
        for (const auto& block : blocks_) {
            if (block != 0) return true;
        }
        return false;
    }
    
    /// Returns true if no bits are set
    [[nodiscard]] bool none() const { return !any(); }
    
    /// Count number of set bits
    [[nodiscard]] size_type count() const noexcept {
        size_type result = 0;
        for (const auto& block : blocks_) {
            result += popcount(block);
        }
        return result;
    }
    
    /// Find first set bit, or npos if none
    [[nodiscard]] IndexType find_first() const {
        for (size_t i = 0; i < blocks_.size(); ++i) {
            if (blocks_[i] != 0) {
                IndexType bit = i * BitsPerBlock + ctz(blocks_[i]);
                return (bit < numBits_) ? bit : npos;
            }
        }
        return npos;
    }
    
    /// Find next set bit after n, or npos if none
    [[nodiscard]] IndexType find_next(IndexType n) const {
        if (n + 1 >= numBits_) return npos;
        return findSetBitAfter(n + 1);
    }
    
    /// Find last set bit, or npos if none
    [[nodiscard]] IndexType find_last() const {
        for (size_t i = blocks_.size(); i > 0; --i) {
            if (blocks_[i - 1] != 0) {
                IndexType bit = (i - 1) * BitsPerBlock + (BitsPerBlock - 1 - clz(blocks_[i - 1]));
                return (bit < numBits_) ? bit : npos;
            }
        }
        return npos;
    }
    
    /// Returns true if this is a subset of a
    [[nodiscard]] bool is_subset_of(const BitSet& a) const {
        size_t minBlocks = std::min(blocks_.size(), a.blocks_.size());
        for (size_t i = 0; i < minBlocks; ++i) {
            if ((blocks_[i] & ~a.blocks_[i]) != 0) return false;
        }
        // Check remaining blocks in this (must all be 0)
        for (size_t i = minBlocks; i < blocks_.size(); ++i) {
            if (blocks_[i] != 0) return false;
        }
        return true;
    }
    
    /// Returns true if this intersects with a
    [[nodiscard]] bool intersects(const BitSet& a) const {
        size_t minBlocks = std::min(blocks_.size(), a.blocks_.size());
        for (size_t i = 0; i < minBlocks; ++i) {
            if ((blocks_[i] & a.blocks_[i]) != 0) return true;
        }
        return false;
    }
    
    // ==================== Auto-resize Operations ====================
    
    /// Resize with reserve doubling strategy
    void resizeWithReserve(size_t newSize) {
        auto reserved = capacity();
        if (reserved > 0 && newSize > reserved) {
            while (newSize > reserved) {
                reserved <<= 1;
            }
            reserve(reserved);
        }
        resize(newSize);
    }
    
    /// Set bit with auto-resize
    void autoResizeSet(size_t pos, bool val = true) {
        if (pos >= size()) {
            resizeWithReserve(pos + 1);
        }
        set(pos, val);
    }
    
    // ==================== Utility ====================
    
    [[nodiscard]] size_t heapBytes() const { return capacity() / 8; }
    
    [[nodiscard]] IndexType backId() const { 
        assert(!empty()); 
        return IndexType{size() - 1}; 
    }
    
    [[nodiscard]] static IndexType beginId() { return IndexType{0}; }
    [[nodiscard]] IndexType endId() const { return IndexType{size()}; }
    
    /// Read-only access to blocks
    [[nodiscard]] const std::vector<BlockType>& bits() const { return blocks_; }
    
private:
    std::vector<BlockType> blocks_;
    size_t numBits_ = 0;
    
    static size_t calcNumBlocks(size_t numBits) noexcept {
        return (numBits + BitsPerBlock - 1) / BitsPerBlock;
    }
    
    static size_t blockIndex(IndexType n) noexcept {
        return n / BitsPerBlock;
    }
    
    static size_t bitIndex(IndexType n) noexcept {
        return n % BitsPerBlock;
    }
    
    static BlockType bitMask(IndexType n) noexcept {
        return BlockType{1} << bitIndex(n);
    }
    
    void resetUnusedBits() {
        if (numBits_ == 0 || blocks_.empty()) return;
        size_t unused = numBits_ % BitsPerBlock;
        if (unused > 0) {
            blocks_.back() &= (BlockType{1} << unused) - 1;
        }
    }
    
    IndexType findSetBitAfter(IndexType start) const {
        if (start >= numBits_) return npos;
        
        size_t blockIdx = blockIndex(start);
        size_t bitIdx = bitIndex(start);
        
        // Check first block (mask off bits before start)
        BlockType masked = blocks_[blockIdx] & (~BlockType{0} << bitIdx);
        if (masked != 0) {
            IndexType bit = blockIdx * BitsPerBlock + ctz(masked);
            return (bit < numBits_) ? bit : npos;
        }
        
        // Check remaining blocks
        for (size_t i = blockIdx + 1; i < blocks_.size(); ++i) {
            if (blocks_[i] != 0) {
                IndexType bit = i * BitsPerBlock + ctz(blocks_[i]);
                return (bit < numBits_) ? bit : npos;
            }
        }
        
        return npos;
    }
    
    // Population count (number of 1 bits)
    static size_t popcount(BlockType x) noexcept {
#if defined(__GNUC__) || defined(__clang__)
        return __builtin_popcountll(x);
#elif defined(_MSC_VER)
        return __popcnt64(x);
#else
        // Fallback implementation
        size_t count = 0;
        while (x) {
            count += x & 1;
            x >>= 1;
        }
        return count;
#endif
    }
    
    // Count trailing zeros
    static size_t ctz(BlockType x) noexcept {
        assert(x != 0);
#if defined(__GNUC__) || defined(__clang__)
        return __builtin_ctzll(x);
#elif defined(_MSC_VER)
        unsigned long index;
        _BitScanForward64(&index, x);
        return index;
#else
        size_t count = 0;
        while ((x & 1) == 0) {
            ++count;
            x >>= 1;
        }
        return count;
#endif
    }
    
    // Count leading zeros
    static size_t clz(BlockType x) noexcept {
        assert(x != 0);
#if defined(__GNUC__) || defined(__clang__)
        return __builtin_clzll(x);
#elif defined(_MSC_VER)
        unsigned long index;
        _BitScanReverse64(&index, x);
        return 63 - index;
#else
        size_t count = 0;
        BlockType mask = BlockType{1} << 63;
        while ((x & mask) == 0) {
            ++count;
            mask >>= 1;
        }
        return count;
#endif
    }
};

// ==================== Binary Operators ====================

inline BitSet operator&(BitSet a, const BitSet& b) { return a &= b; }
inline BitSet operator|(BitSet a, const BitSet& b) { return a |= b; }
inline BitSet operator^(BitSet a, const BitSet& b) { return a ^= b; }
inline BitSet operator-(BitSet a, const BitSet& b) { return a -= b; }

// ==================== SetBitIterator ====================

/**
 * @brief Iterator over set bits in a BitSet
 */
template <typename BS>
class SetBitIteratorT {
public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = typename BS::IndexType;
    using difference_type = std::ptrdiff_t;
    using pointer = const value_type*;
    using reference = value_type;
    
    SetBitIteratorT() : bs_(nullptr), pos_(BS::npos) {}
    
    SetBitIteratorT(const BS& bs, value_type pos) : bs_(&bs), pos_(pos) {}
    
    static SetBitIteratorT begin(const BS& bs) {
        return SetBitIteratorT(bs, bs.find_first());
    }
    
    static SetBitIteratorT end(const BS& bs) {
        return SetBitIteratorT(bs, BS::npos);
    }
    
    value_type operator*() const { return pos_; }
    
    SetBitIteratorT& operator++() {
        pos_ = bs_->find_next(pos_);
        return *this;
    }
    
    SetBitIteratorT operator++(int) {
        auto tmp = *this;
        ++(*this);
        return tmp;
    }
    
    bool operator==(const SetBitIteratorT& other) const {
        return pos_ == other.pos_;
    }
    
    bool operator!=(const SetBitIteratorT& other) const {
        return pos_ != other.pos_;
    }
    
private:
    const BS* bs_;
    value_type pos_;
};

using SetBitIterator = SetBitIteratorT<BitSet>;

// ==================== TypedBitSet ====================

/**
 * @brief Type-safe bit set indexed by Id<T>
 * 
 * Wraps BitSet to provide type-safe indexing with Id types.
 * Prevents accidentally using a VertId to index into a FaceBitSet.
 */
template <typename I>
class TypedBitSet : public BitSet {
public:
    using IndexType = I;
    using BaseType = BitSet;
    
    TypedBitSet() = default;
    
    explicit TypedBitSet(size_t numBits, bool fillValue = false) 
        : BitSet(numBits, fillValue) {}
    
    explicit TypedBitSet(BitSet&& bs) : BitSet(std::move(bs)) {}
    
    // Override methods to use typed index
    [[nodiscard]] bool test(I id) const { 
        return id.valid() && BitSet::test(static_cast<size_t>(id)); 
    }
    
    [[nodiscard]] bool operator[](I id) const { return test(id); }
    
    TypedBitSet& set(I id, bool val = true) {
        assert(id.valid() && static_cast<size_t>(id) < size());
        BitSet::set(static_cast<size_t>(id), val);
        return *this;
    }
    
    TypedBitSet& reset(I id) {
        if (id.valid()) {
            BitSet::reset(static_cast<size_t>(id));
        }
        return *this;
    }
    
    TypedBitSet& flip(I id) {
        assert(id.valid() && static_cast<size_t>(id) < size());
        BitSet::flip(static_cast<size_t>(id));
        return *this;
    }
    
    void autoResizeSet(I id, bool val = true) {
        assert(id.valid());
        BitSet::autoResizeSet(static_cast<size_t>(id), val);
    }
    
    [[nodiscard]] I find_first() const {
        auto pos = BitSet::find_first();
        return (pos == npos) ? I{} : I{static_cast<int>(pos)};
    }
    
    [[nodiscard]] I find_next(I id) const {
        auto pos = BitSet::find_next(static_cast<size_t>(id));
        return (pos == npos) ? I{} : I{static_cast<int>(pos)};
    }
    
    [[nodiscard]] I find_last() const {
        auto pos = BitSet::find_last();
        return (pos == npos) ? I{} : I{static_cast<int>(pos)};
    }
    
    [[nodiscard]] I backId() const { 
        assert(!empty()); 
        return I{static_cast<int>(size() - 1)}; 
    }
    
    [[nodiscard]] static I beginId() { return I{0}; }
    [[nodiscard]] I endId() const { return I{static_cast<int>(size())}; }
    
    // Set operations preserve type
    TypedBitSet& operator&=(const TypedBitSet& b) { 
        BitSet::operator&=(b); 
        return *this; 
    }
    TypedBitSet& operator|=(const TypedBitSet& b) { 
        BitSet::operator|=(b); 
        return *this; 
    }
    TypedBitSet& operator^=(const TypedBitSet& b) { 
        BitSet::operator^=(b); 
        return *this; 
    }
    TypedBitSet& operator-=(const TypedBitSet& b) { 
        BitSet::operator-=(b); 
        return *this; 
    }
};

// Binary operators for TypedBitSet
template <typename I>
TypedBitSet<I> operator&(TypedBitSet<I> a, const TypedBitSet<I>& b) { return a &= b; }
template <typename I>
TypedBitSet<I> operator|(TypedBitSet<I> a, const TypedBitSet<I>& b) { return a |= b; }
template <typename I>
TypedBitSet<I> operator^(TypedBitSet<I> a, const TypedBitSet<I>& b) { return a ^= b; }
template <typename I>
TypedBitSet<I> operator-(TypedBitSet<I> a, const TypedBitSet<I>& b) { return a -= b; }

// ==================== Type Aliases ====================

using FaceBitSet = TypedBitSet<FaceId>;
using VertBitSet = TypedBitSet<VertId>;
using EdgeBitSet = TypedBitSet<EdgeId>;
using UndirectedEdgeBitSet = TypedBitSet<UndirectedEdgeId>;
using NodeBitSet = TypedBitSet<NodeId>;

// Typed BitSet from Id tag
template <typename T>
using TaggedBitSet = TypedBitSet<Id<T>>;

// Typed iterators
using FaceSetBitIterator = SetBitIteratorT<FaceBitSet>;
using VertSetBitIterator = SetBitIteratorT<VertBitSet>;
using EdgeSetBitIterator = SetBitIteratorT<EdgeBitSet>;
using UndirectedEdgeSetBitIterator = SetBitIteratorT<UndirectedEdgeBitSet>;

// ==================== Utility Functions ====================

/// Check if a given face/vertex/edge is in the optional region (or is valid if region is null)
template <typename I>
bool contains(const TypedBitSet<I>* region, I id) {
    return region ? region->test(id) : id.valid();
}

} // namespace meshlib
