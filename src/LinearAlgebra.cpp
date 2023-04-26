#include "LinearAlgebra.h"

std::vector<std::vector<float>> MatrixProduct(const std::vector<std::vector<float>>& left, const std::vector<std::vector<float>>& right)
{
  size_t n = left.size();
  size_t m = left.at(0).size();
  size_t mr= right.size();
  size_t o = right.at(0).size();

  std::vector<std::vector<float>> matrix;
  matrix.resize(n, std::vector<float>(o));
  
  if(m != mr)
  {
    Serial.println("Sizes of matrices are not compatible!! (Product)" + String(m) + " " + String(mr));
  }

  for(size_t i = 0; i < n; i++)
  {
    for(size_t j = 0; j < o; j++)
    {
      float Value = 0;
      for(size_t k = 0; k < m; k++)
      {
        Value += left.at(i).at(k) * right.at(k).at(j);
      }
      matrix.at(i).at(j) = Value;
    }
  }
  return matrix;
}

// helper function to print matrices
void printMatrix(const std::vector<std::vector<float>>& matrix) 
{
  size_t n = matrix.size();
  size_t m = matrix.at(0).size();
  
  for (size_t i = 0; i < n; i++) 
  {
    for (size_t j = 0; j < m; j++) 
    {
      Serial.print(matrix.at(i).at(j), 12);
      Serial.print(" ");
    }
    Serial.println();
  }
  Serial.println();
}

// helper function to get the minor of a matrix
std::vector<std::vector<float>> getMinor(const std::vector<std::vector<float>>& matrix, int rowToRemove, int colToRemove) 
{
    std::vector<std::vector<float>> minor(matrix.size()-1, std::vector<float>(matrix.size()-1));
    int minorRow = 0;
    int minorCol = 0;
    for (uint16_t i = 0; i < matrix.size(); i++) {
        if (i == rowToRemove) continue;
        minorCol = 0;
        for (uint16_t j = 0; j < matrix.size(); j++) {
            if (j == colToRemove) continue;
            minor.at(minorRow).at(minorCol) = matrix.at(i).at(j);
            minorCol++;
        }
        minorRow++;
    }
    return minor;
}

// helper function to calculate the determinant of a matrix
float determinant(const std::vector<std::vector<float>>& matrix) 
{
    if (matrix.size() == 1) {
        return matrix.at(0).at(0);
    }
    float det = 0;
    for (uint16_t i = 0; i < matrix.size(); i++) {
        auto minor = getMinor(matrix, 0, i);
        float sign = (i % 2 == 0) ? 1 : -1;
        det += sign * matrix.at(0).at(i) * determinant(minor);
    }
    return det;
}

// helper function to calculate the cofactor matrix of a matrix
std::vector<std::vector<float>> cofactor(const std::vector<std::vector<float>>& matrix) 
{
    std::vector<std::vector<float>> cofactorMatrix(matrix.size(), std::vector<float>(matrix.size()));
    for (uint16_t i = 0; i < matrix.size(); i++) {
        for (uint16_t j = 0; j < matrix.size(); j++) {
            auto minor = getMinor(matrix, i, j);
            float sign = ((i+j) % 2 == 0) ? 1 : -1;
            cofactorMatrix.at(i).at(j) = sign * determinant(minor);
        }
    }
    return cofactorMatrix;
}

// helper function to calculate the adjugate matrix of a matrix
std::vector<std::vector<float>> adjugate(const std::vector<std::vector<float>>& matrix) 
{
    std::vector<std::vector<float>> cofactorMatrix = cofactor(matrix);
    std::vector<std::vector<float>> adjugateMatrix(matrix.size(), std::vector<float>(matrix.size()));
    for (uint16_t i = 0; i < matrix.size(); i++) {
        for (uint16_t j = 0; j < matrix.size(); j++) {
            adjugateMatrix.at(j).at(i) = cofactorMatrix.at(i).at(j);
        }
    }
    return adjugateMatrix;
}

// main function to calculate the inverse of a matrix
std::vector<std::vector<float>> inverse(const std::vector<std::vector<float>>& matrix) 
{
    auto det = determinant(matrix);
    if (det == 0) {
      std::cerr << "Matrix is not invertible" << std::endl;
    }
    auto adjugateMatrix = adjugate(matrix);
    std::vector<std::vector<float>> inverseMatrix(matrix.size(), std::vector<float>(matrix.size()));
    for (uint16_t i = 0; i < matrix.size(); i++) {
        for (uint16_t j = 0; j < matrix.size(); j++) {
            inverseMatrix.at(i).at(j) = adjugateMatrix.at(i).at(j) / det;
        }
    }
    return inverseMatrix;
}

std::vector<std::vector<float>> transpose(const std::vector<std::vector<float>>& matrixIN)
{
  size_t n = matrixIN.size();
  size_t m = matrixIN.at(0).size();
  
  std::vector<std::vector<float>> matrixOUT;
  matrixOUT.resize(m, std::vector<float>(n));

  for (size_t i = 0; i < m; i++ )
  {
    for (size_t j = 0; j < n; j++)
    {
      matrixOUT.at(i).at(j) = matrixIN.at(j).at(i);
    }
  }
  return matrixOUT;
}

std::vector<std::vector<float>> sum(const std::vector<std::vector<float>>& matrix_1, const std::vector<std::vector<float>>& matrix_2)
{
  size_t n1 = matrix_1.size();
  size_t m1 = matrix_1.at(0).size();
  size_t n2 = matrix_2.size();
  size_t m2 = matrix_2.at(0).size();

  if ( (n1 != n2) || (m1 != m2))
  {
    Serial.println("Sizes of matrices are not compatible!! (sum)" + String(n1)  + " " + String(n2) + " " + String(m1) + " " + String(m2));
  }

  std::vector<std::vector<float>> matrix;
  matrix.resize(n1, std::vector<float>(m1));

  for (size_t i = 0; i < n1; i++)
  {
    for (size_t j = 0; j < m1; j++)
    {
      matrix.at(i).at(j) = matrix_1.at(i).at(j) + matrix_2.at(i).at(j);
    }
  }
  return matrix;
}

std::vector<std::vector<float>> diff(const std::vector<std::vector<float>>& matrix_1, const std::vector<std::vector<float>>& matrix_2)
{
  size_t n1 = matrix_1.size();
  size_t m1 = matrix_1.at(0).size();
  size_t n2 = matrix_2.size();
  size_t m2 = matrix_2.at(0).size();

  if ( (n1 != n2) || (m1 != m2))
  {
    Serial.println("Sizes of matrices are not compatible!! (diff) " + String(n1)  + " " + String(n2) + " " + String(m1) + " " + String(m2));
  }

  std::vector<std::vector<float>> matrix;
  matrix.resize(n1, std::vector<float>(m1));

  for (size_t i = 0; i < n1; i++)
  {
    for (size_t j = 0; j < m1; j++)
    {
      matrix.at(i).at(j) = matrix_1.at(i).at(j) - matrix_2.at(i).at(j);
    }
  }
  return matrix;
}