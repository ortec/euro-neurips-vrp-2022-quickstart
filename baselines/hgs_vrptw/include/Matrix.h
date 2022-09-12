
#ifndef MATRIX_H
#define MATRIX_H

#include <vector>

// Implementation of a matrix in a C++ vector
// This class is used because a flat vector is faster than a vector of vectors which requires two lookup operations rather than one to index a matrix element
class Matrix
{
    int cols_;                  // The number of columns of the matrix
    std::vector<int> data_;     // The vector where all the data is stored (this represents the matrix)

public:
    // Empty constructor: with zero columns and a vector of size zero
    Matrix() : cols_(0), data_(std::vector<int>(0))
    {}

    // Constructor: create a matrix of size dimension by dimension, using a C++ vector of size dimension * dimension 
    Matrix(const int dimension) : cols_(dimension)
    {
        data_ = std::vector<int>(dimension * dimension);
    }

    // Set a value val at position (row, col) in the matrix
    void set(const int row, const int col, const int val)
    {
        data_[cols_ * row + col] = val;
    }

    // Get the value at position (row, col) in the matrix
    int get(const int row, const int col) const
    {
        return data_[cols_ * row + col];
    }
};

#endif
