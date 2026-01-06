#include "lib.h"
#include "api.h"
#include "util.h"

std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> identity_mat = {{
    {{1,0,0,0,0,0,0}},
    {{0,1,0,0,0,0,0}},
    {{0,0,1,0,0,0,0}},
    {{0,0,0,1,0,0,0}},
    {{0,0,0,0,1,0,0}},
    {{0,0,0,0,0,1,0}},
    {{0,0,0,0,0,0,1}}
}};

double average(std::vector<double> list) {
    double sum = 0;
    for (int i = 0; i < list.size(); i++) {
        sum += list[i];
    }
    return sum/list.size();
}

int sgn(float num) {
    if (num > 0) {
        return -1;
    } else {
        return 1;
    }
}

double distance(Point one, Point two) {
    return sqrt(pow((two.x-one.x), 2)+pow((two.y-one.y), 2));
}

double min_angle(double angle) {
    double x = angle;

    // limitations: angles greater than 360 or less than -360
    while (x > 180 || x <= -180) {
        if (x > 180) {
            x = x - 360;
        } else if (x <= -180) {
            x = x + 360;
        }
    }
    return x;
}

Point POI(Point one_line[2], Point two_line[2]) {
    return {1000, 1000};
}

float det_matrix(float** matrix, int n) {
    // Base cases
    if (n == 1) {
        return matrix[0][0];
    }
    if (n == 2) {
        return matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
    }
    
    float result = 0;
    
    // Allocate memory for submatrix
    float** submatrix = new float*[n-1];
    for (int i = 0; i < n-1; i++) {
        submatrix[i] = new float[n-1];
    }
    
    // Calculate determinant using Laplace expansion along first row
    for (int col = 0; col < n; col++) {
        // Create (n-1)x(n-1) submatrix by excluding row 0 and column col
        int sub_row = 0;
        for (int i = 1; i < n; i++) {
            int sub_col = 0;
            for (int j = 0; j < n; j++) {
                if (j != col) {
                    submatrix[sub_row][sub_col] = matrix[i][j];
                    sub_col++;
                }
            }
            sub_row++;
        }
        
        // Calculate cofactor
        float cofactor = matrix[0][col] * det_matrix(submatrix, n-1);
        if (col % 2 == 1) cofactor = -cofactor;
        
        result += cofactor;
    }
    
    // Clean up memory
    for (int i = 0; i < n-1; i++) {
        delete[] submatrix[i];
    }
    delete[] submatrix;
    
    return result;
}

float det_mat_77(std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> matrix) {
    // Convert std::array to array of pointers for compatibility
    float** mat = new float*[7];
    for (int i = 0; i < 7; i++) {
        mat[i] = matrix[i].data();
    }
    
    float result = det_matrix(mat, 7);
    delete[] mat;
    return result;
}

float trace_mat_77(std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> matrix) {
    return matrix[0][0] + matrix[1][1] + matrix[2][2] + matrix[3][3] + matrix[4][4] + matrix[5][5] + matrix[6][6]; 
}

std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> add_mat(std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> one, std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> two) {
    std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> result;
    
    for (int i=0;i<STATE_DIMENSIONS;i++) {
        for (int j=0;j<STATE_DIMENSIONS;j++) {
            result[i][j] = one[i][j] + two[i][j];
        }
    }

    return result;
}

std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> multiply_const_mat(std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> matrix, float scale) {
    std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> result;

    for (int i=0;i<STATE_DIMENSIONS;i++) {
        for (int j=0;j<STATE_DIMENSIONS;j++) {
            result[i][j] = matrix[i][j] * scale;
        }
    }
    
    return result;
}

float deg_to_rad(float angle) {
    return M_PI*angle/180;
}

// Additional matrix operations for UKF
std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> subtract_mat_77(
    std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> one, 
    std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> two) {
    
    std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> result;
    
    for (int i = 0; i < STATE_DIMENSIONS; i++) {
        for (int j = 0; j < STATE_DIMENSIONS; j++) {
            result[i][j] = one[i][j] - two[i][j];
        }
    }
    
    return result;
}

std::array<std::array<float, MEASUREMENT_DIMENSIONS>, MEASUREMENT_DIMENSIONS> inverse_mat_66(
    std::array<std::array<float, MEASUREMENT_DIMENSIONS>, MEASUREMENT_DIMENSIONS> matrix) {
    
    std::array<std::array<float, MEASUREMENT_DIMENSIONS>, MEASUREMENT_DIMENSIONS> result;
    std::array<std::array<float, MEASUREMENT_DIMENSIONS>, MEASUREMENT_DIMENSIONS * 2> augmented;
    
    // Create augmented matrix [A | I]
    for (int i = 0; i < MEASUREMENT_DIMENSIONS; i++) {
        for (int j = 0; j < MEASUREMENT_DIMENSIONS; j++) {
            augmented[i][j] = matrix[i][j];
            augmented[i][j + MEASUREMENT_DIMENSIONS] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    // Gaussian elimination with partial pivoting
    for (int i = 0; i < MEASUREMENT_DIMENSIONS; i++) {
        // Find pivot
        int max_row = i;
        for (int k = i + 1; k < MEASUREMENT_DIMENSIONS; k++) {
            if (fabs(augmented[k][i]) > fabs(augmented[max_row][i])) {
                max_row = k;
            }
        }
        
        // Swap rows if needed
        if (max_row != i) {
            for (int k = 0; k < MEASUREMENT_DIMENSIONS * 2; k++) {
                float temp = augmented[i][k];
                augmented[i][k] = augmented[max_row][k];
                augmented[max_row][k] = temp;
            }
        }
        
        // Make diagonal element 1
        float pivot = augmented[i][i];
        if (fabs(pivot) < 1e-10) {
            // Matrix is singular, return identity as fallback
            for (int row = 0; row < MEASUREMENT_DIMENSIONS; row++) {
                for (int col = 0; col < MEASUREMENT_DIMENSIONS; col++) {
                    result[row][col] = (row == col) ? 1.0f : 0.0f;
                }
            }
            return result;
        }
        
        for (int k = 0; k < MEASUREMENT_DIMENSIONS * 2; k++) {
            augmented[i][k] /= pivot;
        }
        
        // Eliminate column
        for (int k = 0; k < MEASUREMENT_DIMENSIONS; k++) {
            if (k != i) {
                float factor = augmented[k][i];
                for (int j = 0; j < MEASUREMENT_DIMENSIONS * 2; j++) {
                    augmented[k][j] -= factor * augmented[i][j];
                }
            }
        }
    }
    
    // Extract result matrix
    for (int i = 0; i < MEASUREMENT_DIMENSIONS; i++) {
        for (int j = 0; j < MEASUREMENT_DIMENSIONS; j++) {
            result[i][j] = augmented[i][j + MEASUREMENT_DIMENSIONS];
        }
    }
    
    return result;
}

std::array<std::array<float, STATE_DIMENSIONS>, MEASUREMENT_DIMENSIONS> multiply_mat_76_66(
    std::array<std::array<float, STATE_DIMENSIONS>, MEASUREMENT_DIMENSIONS> mat1,
    std::array<std::array<float, MEASUREMENT_DIMENSIONS>, MEASUREMENT_DIMENSIONS> mat2) {
    
    std::array<std::array<float, STATE_DIMENSIONS>, MEASUREMENT_DIMENSIONS> result;
    
    for (int i = 0; i < STATE_DIMENSIONS; i++) {
        for (int j = 0; j < MEASUREMENT_DIMENSIONS; j++) {
            result[i][j] = 0;
            for (int k = 0; k < MEASUREMENT_DIMENSIONS; k++) {
                result[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }
    
    return result;
}

std::array<std::array<float, MEASUREMENT_DIMENSIONS>, STATE_DIMENSIONS> transpose_mat_76(
    std::array<std::array<float, STATE_DIMENSIONS>, MEASUREMENT_DIMENSIONS> matrix) {
    
    std::array<std::array<float, MEASUREMENT_DIMENSIONS>, STATE_DIMENSIONS> result;
    
    for (int i = 0; i < MEASUREMENT_DIMENSIONS; i++) {
        for (int j = 0; j < STATE_DIMENSIONS; j++) {
            result[i][j] = matrix[j][i];
        }
    }
    
    return result;
}

std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> multiply_mat_76_67(
    std::array<std::array<float, STATE_DIMENSIONS>, MEASUREMENT_DIMENSIONS> mat1,
    std::array<std::array<float, MEASUREMENT_DIMENSIONS>, STATE_DIMENSIONS> mat2) {
    
    std::array<std::array<float, STATE_DIMENSIONS>, STATE_DIMENSIONS> result;
    
    for (int i = 0; i < STATE_DIMENSIONS; i++) {
        for (int j = 0; j < STATE_DIMENSIONS; j++) {
            result[i][j] = 0;
            for (int k = 0; k < MEASUREMENT_DIMENSIONS; k++) {
                result[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }
    
    return result;
}
