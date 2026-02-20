#pragma once

#include "kaizen/core/types.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace kaizen {

struct DenseMat {
    u32 rows = 0;
    u32 cols = 0;
    std::vector<f32> data;

    DenseMat() = default;
    DenseMat(u32 r, u32 c) : rows(r), cols(c), data(r * c, 0.0f) {}

    f32& operator()(u32 r, u32 c) { return data[r * cols + c]; }
    f32 operator()(u32 r, u32 c) const { return data[r * cols + c]; }

    void set_zero() { std::fill(data.begin(), data.end(), 0.0f); }
};

// C = A * B
inline DenseMat mat_mul(const DenseMat& A, const DenseMat& B) {
    DenseMat C(A.rows, B.cols);
    for (u32 i = 0; i < A.rows; ++i) {
        for (u32 k = 0; k < A.cols; ++k) {
            f32 a = A(i, k);
            if (a == 0.0f) continue;
            for (u32 j = 0; j < B.cols; ++j) {
                C(i, j) += a * B(k, j);
            }
        }
    }
    return C;
}

// B = A^T
inline DenseMat mat_transpose(const DenseMat& A) {
    DenseMat B(A.cols, A.rows);
    for (u32 i = 0; i < A.rows; ++i) {
        for (u32 j = 0; j < A.cols; ++j) {
            B(j, i) = A(i, j);
        }
    }
    return B;
}

// A += s * I (adds scalar to diagonal)
inline void mat_add_scaled_identity(DenseMat& A, f32 s) {
    u32 n = std::min(A.rows, A.cols);
    for (u32 i = 0; i < n; ++i) {
        A(i, i) += s;
    }
}

// y = A * x
inline std::vector<f32> mat_vec_mul(const DenseMat& A, const std::vector<f32>& x) {
    std::vector<f32> y(A.rows, 0.0f);
    for (u32 i = 0; i < A.rows; ++i) {
        for (u32 j = 0; j < A.cols; ++j) {
            y[i] += A(i, j) * x[j];
        }
    }
    return y;
}

// Solve Ax = b via Gaussian elimination with partial pivoting.
// A is modified in place. Returns x.
inline std::vector<f32> solve_linear(DenseMat A, std::vector<f32> b) {
    u32 n = A.rows;
    for (u32 col = 0; col < n; ++col) {
        // Partial pivot
        u32 max_row = col;
        f32 max_val = std::abs(A(col, col));
        for (u32 row = col + 1; row < n; ++row) {
            f32 v = std::abs(A(row, col));
            if (v > max_val) {
                max_val = v;
                max_row = row;
            }
        }
        if (max_row != col) {
            for (u32 j = 0; j < n; ++j) {
                std::swap(A(col, j), A(max_row, j));
            }
            std::swap(b[col], b[max_row]);
        }

        f32 pivot = A(col, col);
        if (std::abs(pivot) < 1e-12f) continue;

        for (u32 row = col + 1; row < n; ++row) {
            f32 factor = A(row, col) / pivot;
            for (u32 j = col; j < n; ++j) {
                A(row, j) -= factor * A(col, j);
            }
            b[row] -= factor * b[col];
        }
    }

    // Back substitution
    std::vector<f32> x(n, 0.0f);
    for (i32 i = static_cast<i32>(n) - 1; i >= 0; --i) {
        f32 sum = b[static_cast<u32>(i)];
        for (u32 j = static_cast<u32>(i) + 1; j < n; ++j) {
            sum -= A(static_cast<u32>(i), j) * x[j];
        }
        f32 diag = A(static_cast<u32>(i), static_cast<u32>(i));
        x[static_cast<u32>(i)] = (std::abs(diag) > 1e-12f) ? sum / diag : 0.0f;
    }
    return x;
}

} // namespace kaizen
