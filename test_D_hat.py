import numpy as np

def create_convolution_matrix_py(P, C_hor, A, B, C):
    """
    Генерирует матрицу свертки D_hat с горизонтами P (предсказание) и C_hor (управление).
    """
    n_out = C.shape[0] # Количество выходов (строк в C)
    m_in = B.shape[1]  # Количество входов (столбцов в B)

    num_rows = P * n_out
    num_cols = C_hor * m_in
    
    D_hat = np.zeros((num_rows, num_cols))

    for i in range(P):       # Строки блоков (0 до P-1)
        for j in range(C_hor):  # Столбцы блоков (0 до C_hor-1)
            
            if j > i:
                continue 
            
            A_power = np.linalg.matrix_power(A, i - j)
            block_value = C @ A_power @ B
            
            start_row = i * n_out
            start_col = j * m_in
            D_hat[start_row:start_row + n_out, 
                  start_col:start_col + m_in] = block_value

    return D_hat

# --- Параметры задачи ---
p = 10  # Горизонт предсказания (P)
c = 5   # Горизонт управления (C_hor)
n_out = 3 
m_in = 1

# --- Ваши матрицы из лога ---

# A: 3x3
A_np = np.array([
    [1.0, 0.06, 0.0],
    [0.0, 0.7,  0.0],
    [0.0, -5.0, 0.0]
])

# B: 3x1
B_np = np.array([
    [0.0],
    [0.3],
    [5.0]
])

# C: 3x3 (Единичная матрица I3)
C_np = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0]
])

# --- Генерация матрицы D_hat ---
D_hat_np = create_convolution_matrix_py(p, c, A_np, B_np, C_np)

print(f"Сгенерирована матрица D_hat размеров {D_hat_np.shape}:")
# np.set_printoptions(precision=4, suppress=True, linewidth=150) # Для красивого вывода
print(D_hat_np)

print("\n--- Проверка ключевых блоков ---")

# 1. Верхний левый блок (CB)
top_left = D_hat_np[0:n_out, 0:m_in]
print(f"Верхний левый блок (CB):\n{top_left}")

# 2. Нижний правый ненулевой блок (CA^(p-c)B = CA^5B)
bottom_right_block = D_hat_np[(p-1)*n_out:p*n_out, (c-1)*m_in:c*m_in]
print(f"Нижний правый блок (CA^5B):\n{bottom_right_block}")
