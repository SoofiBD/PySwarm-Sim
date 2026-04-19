import numpy as np

class Matrix:
    @staticmethod
    def MatrixZeros(rows: int, cols: int):
        return np.zeros((rows, cols), dtype=np.float64)

    @staticmethod
    def MatrixCreate(rows: int, cols: int):
        return np.zeros((rows, cols), dtype=np.float64)

    @staticmethod
    def MatrixProduct(matrixA, matrixB):
        # Using numpy's matrix multiplication
        return np.matmul(matrixA, matrixB)
    
    @staticmethod
    def MatrixAsString(matrix) -> str:
        # Improved formatting for the adjacency matrix display
        if not isinstance(matrix, np.ndarray):
            matrix = np.array(matrix)
        
        # Use numpy's array_str but with custom formatting if needed
        # and match the manual padding from the old code
        rows, cols = matrix.shape
        s = ""
        for i in range(rows):
            for j in range(cols):
                val = str(round(matrix[i][j], 2))
                s += val.ljust(10) + " "
            s += "\n"
        return s
