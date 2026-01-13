#pragma once

/**
 * @file Expected.h
 * @brief Expected type for error handling
 * 
 * Industrial-strength port of MeshLib's MRExpected.h providing
 * Result<T, E> type for explicit error handling without exceptions.
 */

#include "meshlib/config.h"
#include <variant>
#include <string>
#include <optional>
#include <stdexcept>

namespace meshlib {

/**
 * @brief Expected type - either a value or an error
 * 
 * Similar to std::expected (C++23), provides explicit error handling.
 * 
 * Usage:
 * @code
 * Expected<Mesh, std::string> loadMesh(const std::string& path) {
 *     if (error) return unexpected("Failed to load");
 *     return mesh;
 * }
 * 
 * auto result = loadMesh("test.obj");
 * if (result) {
 *     Mesh& mesh = *result;
 * } else {
 *     std::cerr << result.error() << std::endl;
 * }
 * @endcode
 * 
 * @tparam T Value type
 * @tparam E Error type (default std::string)
 */
template <typename T, typename E = std::string>
class Expected {
public:
    // ==================== Construction ====================
    
    /// Construct with value
    Expected(const T& value) : data_(value) {}
    Expected(T&& value) : data_(std::move(value)) {}
    
    /// Construct with error (use unexpected() helper)
    template <typename U>
    Expected(const std::variant<std::monostate, U>& err) 
        : data_(std::get<U>(err)) {}
    
    /// Copy/move constructors
    Expected(const Expected&) = default;
    Expected(Expected&&) = default;
    Expected& operator=(const Expected&) = default;
    Expected& operator=(Expected&&) = default;
    
    // ==================== Observers ====================
    
    /// Check if contains a value
    bool has_value() const noexcept {
        return std::holds_alternative<T>(data_);
    }
    
    /// Implicit bool conversion
    explicit operator bool() const noexcept {
        return has_value();
    }
    
    /// Check if contains an error
    bool has_error() const noexcept {
        return std::holds_alternative<E>(data_);
    }
    
    // ==================== Value Access ====================
    
    /// Get reference to value (throws if error)
    T& value() & {
        if (!has_value()) {
            throw std::runtime_error("Expected contains error");
        }
        return std::get<T>(data_);
    }
    
    const T& value() const& {
        if (!has_value()) {
            throw std::runtime_error("Expected contains error");
        }
        return std::get<T>(data_);
    }
    
    T&& value() && {
        if (!has_value()) {
            throw std::runtime_error("Expected contains error");
        }
        return std::get<T>(std::move(data_));
    }
    
    /// Dereference operator
    T& operator*() & { return value(); }
    const T& operator*() const& { return value(); }
    T&& operator*() && { return std::move(*this).value(); }
    
    /// Arrow operator
    T* operator->() { return &value(); }
    const T* operator->() const { return &value(); }
    
    /// Get value or default
    template <typename U>
    T value_or(U&& default_value) const& {
        return has_value() ? std::get<T>(data_) : static_cast<T>(std::forward<U>(default_value));
    }
    
    template <typename U>
    T value_or(U&& default_value) && {
        return has_value() ? std::get<T>(std::move(data_)) : static_cast<T>(std::forward<U>(default_value));
    }
    
    // ==================== Error Access ====================
    
    /// Get reference to error (throws if value)
    E& error() & {
        if (!has_error()) {
            throw std::runtime_error("Expected contains value, not error");
        }
        return std::get<E>(data_);
    }
    
    const E& error() const& {
        if (!has_error()) {
            throw std::runtime_error("Expected contains value, not error");
        }
        return std::get<E>(data_);
    }
    
    // ==================== Monadic Operations ====================
    
    /// Transform value if present
    template <typename F>
    auto map(F&& f) const -> Expected<decltype(f(std::declval<T>())), E> {
        using U = decltype(f(std::declval<T>()));
        if (has_value()) {
            return Expected<U, E>(f(std::get<T>(data_)));
        }
        return Expected<U, E>(std::variant<std::monostate, E>(std::get<E>(data_)));
    }
    
    /// Chain operation that may fail
    template <typename F>
    auto and_then(F&& f) const -> decltype(f(std::declval<T>())) {
        using Result = decltype(f(std::declval<T>()));
        if (has_value()) {
            return f(std::get<T>(data_));
        }
        return Result(std::variant<std::monostate, E>(std::get<E>(data_)));
    }
    
    /// Transform error if present
    template <typename F>
    auto map_error(F&& f) const -> Expected<T, decltype(f(std::declval<E>()))> {
        using U = decltype(f(std::declval<E>()));
        if (has_value()) {
            return Expected<T, U>(std::get<T>(data_));
        }
        return Expected<T, U>(std::variant<std::monostate, U>(f(std::get<E>(data_))));
    }
    
private:
    std::variant<T, E> data_;
    
    // Allow unexpected to construct error state
    template <typename U>
    friend class Unexpected;
    
    // Private constructor for error state
    struct ErrorTag {};
    Expected(ErrorTag, const E& err) : data_(err) {}
    Expected(ErrorTag, E&& err) : data_(std::move(err)) {}
    
    template <typename T2, typename E2>
    friend class Expected;
};

/**
 * @brief Wrapper for creating unexpected (error) values
 */
template <typename E>
class Unexpected {
public:
    explicit Unexpected(const E& error) : error_(error) {}
    explicit Unexpected(E&& error) : error_(std::move(error)) {}
    
    const E& error() const& { return error_; }
    E&& error() && { return std::move(error_); }
    
    /// Convert to Expected
    template <typename T>
    operator Expected<T, E>() const {
        return Expected<T, E>(typename Expected<T, E>::ErrorTag{}, error_);
    }
    
private:
    E error_;
};

/// Helper function to create unexpected values
template <typename E>
Unexpected<typename std::decay<E>::type> unexpected(E&& error) {
    return Unexpected<typename std::decay<E>::type>(std::forward<E>(error));
}

// ==================== Void Specialization ====================

/**
 * @brief Expected<void, E> specialization for operations without return value
 */
template <typename E>
class Expected<void, E> {
public:
    /// Construct success state
    Expected() : error_() {}
    
    /// Construct error state
    Expected(const Unexpected<E>& err) : error_(err.error()) {}
    
    /// Check if successful
    bool has_value() const noexcept { return !error_.has_value(); }
    explicit operator bool() const noexcept { return has_value(); }
    bool has_error() const noexcept { return error_.has_value(); }
    
    /// Get error
    const E& error() const { return *error_; }
    
private:
    std::optional<E> error_;
};

// ==================== Type Aliases ====================

/// Common result type with string error
template <typename T>
using Result = Expected<T, std::string>;

/// Void result for operations that can fail
using VoidResult = Expected<void, std::string>;

// ==================== Error Codes ====================

/**
 * @brief Standard error codes for mesh operations
 */
enum class MeshError {
    None = 0,
    InvalidInput,
    EmptyMesh,
    NonManifold,
    DegenerateElement,
    SelfIntersection,
    FileNotFound,
    ParseError,
    IOError,
    OutOfMemory,
    Cancelled,
    Unknown
};

/// Convert error code to string
inline const char* toString(MeshError err) {
    switch (err) {
        case MeshError::None: return "No error";
        case MeshError::InvalidInput: return "Invalid input";
        case MeshError::EmptyMesh: return "Empty mesh";
        case MeshError::NonManifold: return "Non-manifold mesh";
        case MeshError::DegenerateElement: return "Degenerate element";
        case MeshError::SelfIntersection: return "Self-intersection";
        case MeshError::FileNotFound: return "File not found";
        case MeshError::ParseError: return "Parse error";
        case MeshError::IOError: return "I/O error";
        case MeshError::OutOfMemory: return "Out of memory";
        case MeshError::Cancelled: return "Operation cancelled";
        default: return "Unknown error";
    }
}

/// Expected with MeshError
template <typename T>
using MeshResult = Expected<T, MeshError>;

} // namespace meshlib
