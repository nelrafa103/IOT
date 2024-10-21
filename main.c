#include <stdio.h>

#define SIZE 36

void multiplicarMatrices(int a[SIZE][SIZE], int b[SIZE][SIZE], int resultado[SIZE][SIZE]) {
    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            resultado[i][j] = 0;  
            for (int k = 0; k < SIZE; k++) {
                resultado[i][j] += a[i][k] * b[k][j];  
            }
        }
    }
}

void imprimirMatriz(int matriz[SIZE][SIZE]) {
    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            printf("%d ", matriz[i][j]);
        }
        printf("\n");
    }
}

int main() {
    int matriz_a[SIZE][SIZE], matriz_b[SIZE][SIZE], resultado[SIZE][SIZE];

   
    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            matriz_a[i][j] = i + j; 
            matriz_b[i][j] = i - j; 
        }
    }

    
    multiplicarMatrices(matriz_a, matriz_b, resultado);

 
    printf("Resultado de la multiplicación de las matrices:\n");
    imprimirMatriz(resultado);

    return 0;
}