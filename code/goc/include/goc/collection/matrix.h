//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#ifndef GOC_COLLECTION_MATRIX_H
#define GOC_COLLECTION_MATRIX_H

#include <iostream>
#include <vector>

#include "goc/lib/json.hpp"
#include "goc/print/printable.h"
#include "goc/print/print_utils.h"

namespace goc
{
// Represents a matrix of dimension rxc of elements of type T.
// Precondition: r>=0, c>=0.
template<typename T>
class Matrix : public Printable
{
public:
	// Creates an empty matrix.
	Matrix(size_t row_count=0, size_t col_count=0)
		: matrix_(row_count, std::vector<T>(col_count)), row_count_(row_count), col_count_(col_count)
	{ }

	// Creates a matrix with row_count rows, col_count columns, and all cells with the default_element.
	Matrix(size_t row_count, size_t col_count, const T& default_element)
		: matrix_(row_count, std::vector<T>(col_count, default_element)), row_count_(row_count), col_count_(col_count)
	{ }

	// Returns: the number of rows.
	size_t row_count() const
	{
		return row_count_;
	}

	// Returns: the number of columns.
	size_t column_count() const
	{
		return col_count_;
	}

	// Returns: the number of cells.
	size_t size() const
	{
		return row_count_ * col_count_;
	}

	// Returns: the specified row.
	std::vector<T>& operator[](size_t row)
	{
		return matrix_[row];
	}

	// Returns: the specified row.
	const std::vector<T>& operator[](size_t row) const
	{
		return matrix_[row];
	}

	// Returns: the specified cell value.
	T& operator()(size_t row, size_t col)
	{
		return matrix_[row][col];
	}

	// Returns: the specified cell value.
	const T& operator()(size_t row, size_t col) const
	{
		return matrix_[row][col];
	}

	// Returns: the specified cell value.
	const T& at(size_t row, size_t col) const
	{
		return matrix_.at(row).at(col);
	}

	// Clears the content of the matrix by setting the default value of T to each cell.
	void clear()
	{
		matrix_ = std::vector<std::vector<T>>(row_count_, std::vector<T>(col_count_));
	}

	//Returns the transposed matrix
	Matrix transpose() const
	{
		Matrix m(column_count(), row_count());
		for(auto i = 0ul; i < column_count(); ++i)
			for(auto j = 0ul; j < row_count(); ++j)
				m[i][j] = matrix_[j][i];
		return m;
	}

	// Prints the matrix.
	virtual void Print(std::ostream& os) const
	{
		os << matrix_;
	}

private:
	std::vector<std::vector<T>> matrix_;
	size_t row_count_, col_count_;
};

template<typename T>
void from_json(const nlohmann::json& j, Matrix<T>& matrix)
{
	size_t r = j.size();
	size_t c = r > 0 ? j[0].size() : 0;
	matrix = Matrix<T>(r, c);
	for (auto i = 0ul; i < r; ++i)
		for (auto k = 0ul; k < c; ++k)
			matrix[i][k] = j[i][k];
}

template<typename T>
void to_json(nlohmann::json& j, const Matrix<T>& matrix)
{
	j = std::vector<nlohmann::json>();
	for (auto r = 0ul; r < matrix.row_count(); ++r)
	{
		j.push_back(std::vector<nlohmann::json>());
		for (auto c = 0ul; c < matrix.column_count(); ++c)
		{
			j.back().push_back((T)matrix[r][c]);
		}
	}
}

} // namespace goc

#endif //GOC_COLLECTION_MATRIX_H
