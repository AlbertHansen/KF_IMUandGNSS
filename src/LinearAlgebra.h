#ifndef LINEAR_ALGEBRA_H_
#define LINEAR_ALGEBRA_H_

#include <Arduino.h>
#include <iostream>
#include <vector>

void printMatrix(const std::vector<std::vector<float>>& matrix);
float determinant(const std::vector<std::vector<float>>& matrix);
std::vector<std::vector<float>> getMinor(       const std::vector<std::vector<float>>& matrix, int rowToRemove, int colToRemove);
std::vector<std::vector<float>> cofactor(       const std::vector<std::vector<float>>& matrix);
std::vector<std::vector<float>> adjugate(       const std::vector<std::vector<float>>& matrix);
std::vector<std::vector<float>> inverse(        const std::vector<std::vector<float>>& matrix);
std::vector<std::vector<float>> transpose(      const std::vector<std::vector<float>>& matrixIN);
std::vector<std::vector<float>> sum(            const std::vector<std::vector<float>>& matrix_1, const std::vector<std::vector<float>>& matrix_2);
std::vector<std::vector<float>> diff(           const std::vector<std::vector<float>>& matrix_1, const std::vector<std::vector<float>>& matrix_2);
std::vector<std::vector<float>> MatrixProduct(  const std::vector<std::vector<float>>& left, const std::vector<std::vector<float>>& right);
#endif