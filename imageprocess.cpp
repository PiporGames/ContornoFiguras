/***************************************************************************
 *   Copyright (C) 2023 by pilar                                           *
 *   pilarb@unex.es                                                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include "imageprocess.h"


void imageprocess::suavizado(unsigned char * imagenO, unsigned char * imagenD, int W, int H, int * kernel, int N)
{
    // %0 = imagenO; %1 = imagenD; %2 = W; %3 = H; %4 = kernel; %5 = N

    asm volatile(

        "mov %0, %%rsi;" // imagenO
        "mov %1, %%rdi;" // imagenD
        "mov $0, %%r8d;" //y
    "bSuavizadoY:"
        "mov $0, %%r9d;" //x
        "bSuavizadoX:"
            "mov %4, %%rbx;" //kernel
            "mov $0, %%eax;" //acum
            "mov $0, %%ecx;" //totalK
            "mov %5, %%r10d;"
            "neg %%r10d;" //j
            "bSuavizadoKernelJ:"
                "mov %5, %%r11d;"
                "neg %%r11d;" //i
                "bSuavizadoKernelI:"
                    "mov %%r8d, %%r12d;"
                    "add %%r10d, %%r12d;" //y+j
                    "mov %%r9d, %%r13d;"
                    "add %%r11d, %%r13d;" //x+y
                    "cmp $0, %%r12d;"
                    "jl sigKernelI;"
                    "cmp %3, %%r12d;"
                    "jge sigKernelI;"
                    "cmp $0, %%r13d;"
                    "jl sigKernelI;"
                    "cmp %2, %%r13d;"
                    "jge sigKernelI;"

                    "mov %2, %%r14d;"
                    "imul %%r12d, %%r14d;"
                    "add %%r13d, %%r14d;"
                    "movsx %%r14d, %%r14;"
                    "movzbl (%%rsi, %%r14), %%edx;"

                    "imull (%%rbx), %%edx;"
                    "add %%edx, %%eax;"
                    "add (%%rbx), %%ecx;"

                "sigKernelI:"
                    "add $4, %%rbx;"
                    "inc %%r11d;"
                    "cmp %5, %%r11d;"
                    "jle bSuavizadoKernelI;"
            "sigKernelJ:"
                "inc %%r10d;"
                "cmp %5, %%r10d;"
                "jle bSuavizadoKernelJ;"
            "mov $0, %%edx;"
            "div %%ecx;" // acum/kernel_t
            "mov %%r8d, %%r14d;"
            "imul %2, %%r14d;"
            "add %%r9d, %%r14d;"
            "movsx %%r14d, %%r14;"
            "mov %%al, (%%rdi, %%r14);"
            "inc %%r9d;"
            "cmp %2, %%r9d;"
            "jl bSuavizadoX;"
        "inc %%r8d;"
        "cmp %3, %%r8d;"
        "jl bSuavizadoY;"


        :
        : "m" (imagenO), "m" (imagenD), "m" (W), "m" (H), "m" (kernel), "m" (N)
        : "%rax", "%rbx", "%rcx", "%rdx", "%rsi", "%rdi", "%r8", "%r9", "%r10", "%r11", "%r12",
          "%r13", "%r14", "memory"

    );


}

void imageprocess::umbralizar(unsigned char * imagenO, unsigned char * imagenD, unsigned char umbral, int W, int H)
{
    // %0 = imagenO; %1 = imagenD; %2 = umbral; %3 = W; %4 = H

    // IMPORTANTE RECORDAR: los parámetros son INTS, y solo funcionan cuando son guardados en regs de 32b (ej ecx, r9d)
    // TODAS LAS DIRECCIONES DE PUNTEROS EN SYSx64 son de 64b.
    asm volatile(
        "mov %0, %%rsi;" // ESI = imagenO
        "mov %1, %%rdi;" // EDI = imagenD
        "mov %2, %%r8b;" // R8 = umbral
        "mov %3, %%r9d;" // R9 = W
        "mov %4, %%r10d;" // R10 = H
        "xor %%rdx, %%rdx;" // EDX = 0 (negro)

        "Umbral_L1:"
        "mov %%r9d, %%ecx;" // ECX = Cols iniciales (reiniciar)
        "Umbral_L2:"

        "cmpb %%r8b, (%%rsi);"
        "jb noUmbral;"
        "movb $255, (%%rdi);"
        "jmp finUmbral;"
        "noUmbral:"
        "movb %%dl, (%%rdi);" // Poner a negro el pixel
        "finUmbral:"
        "inc %%rdi;"
        "inc %%rsi;"
        "dec %%ecx;" // decrementar el contador de columnas
        "cmp $0, %%ecx;"
        "jne Umbral_L2;" // bucle para cada columna
        "dec %%r10d;" // decrementar el contador de filas
        "cmp $0, %%r10d;"
        "jne Umbral_L1;" // bucle para cada fila
        :
        : "m" (imagenO), "m" (imagenD), "m" (umbral), "m" (W), "m" (H)
        : "memory", "%rcx", "%rdx", "%rdi", "%r8", "%r9", "%r10"
    );

}

void imageprocess::calculoGradientes(unsigned char* imagen, Gradient* gradientes, int W, int H)
{
    int kernel_x[3][3] = {{-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1}};
    int * sobel_x = (int *) &kernel_x;
    int kernel_y[3][3] = {{-1, -2, -1}, {0, 0, 0}, {1, 2, 1}};
    int *  sobel_y = (int *) &kernel_y;

//    // %0 = imagen; %1 = gradientes; %2 = W; %3 = H; %4 = sobel_x; %5 = sobel_y

// Código en C:
//    unsigned char * dirImagenO = imagen;
//    Gradient* dirGradientes = gradientes;
//    for(int y = 0; y < H; y++)
//    {
//        for(int x = 0; x < W; x++)
//        {
//            int * dirSobelX = sobel_x;
//            int * dirSobelY = sobel_y;
//            int gradX = 0;
//            int gradY = 0;
//            for(int j = -1; j <= 1; j++)
//            {
//                for(int i = -1; i <= 1; i++)
//                {
//                    if((x+i) >= 0 && (x+i) < W && (y+j) >= 0 && (y+j) < H)
//                    {
//                        int offsetPixel = (y + j) * W + (x + i);
//                        unsigned char * p = dirImagenO;
//                        p += offsetPixel;
//                        gradX += *p * *dirSobelX;
//                        gradY += *p * *dirSobelY;
//                    }
//                    dirSobelX++;
//                    dirSobelY++;
//                }
//            }
//            int offsetPixel2 = y * W + x;
//            Gradient * p = dirGradientes;
//            p += offsetPixel2;
//            p->dx = gradX;
//            p->dy = gradY;
//        }
//    }


// NOTAS IMPORTANTES:
// Por si no quedaba claro antes, RECUERDA que los parámetros son INTS (32b). Los bucles explorarán si no se utilizan registros de 32b.
// Es posible utilizar parámetros estáticos directamente desde su %n sin usar registros auxiliares, siempre que se cumpla lo anterior.
// Recuerda que no puedes simplemente coger cachos más pequeños de los registros directamente, aunque sepas que solo se ha utilizado la parte baja.
// Para convertir dichos registros se utiliza movzAB, A siendo el origen y B el destino (hablando de tamaños)
// ej. 8b a 32b movzbl
    asm volatile(
        "mov %0, %%rsi;" // RSI = imagen (64)
        "mov %1, %%rdi;" // EDI = gradientes (64)

        "xor %%r12, %%r12;" // R12 = Y (reiniciar) L1
        "Gradientes_L1:"
        "xor %%r13, %%r13;" // R13 = X (reiniciar) L2
        "Gradientes_L2:"

        "mov %4, %%r10;" // R10 = sobel_x (64)
        "mov %5, %%r11;" // R11 = sobel_y (64)
        "mov $0, %%r8d;" // R8 = gradX (32)
        "mov $0, %%r9d;" // R9 = gradY (32)

        "mov $-1, %%r14d;" // R14 = J (reiniciar) L3
        "Gradientes_L3:"
        "mov $-1, %%r15d;" // R15 = I (reiniciar) L4
        "Gradientes_L4:"

        //Comienzo del IF (utilizamos EDX como auxiliar)
        "mov %%r13d, %%edx;"
        "add %%r15d, %%edx;"
        "cmp $0, %%edx;"
        "jl Gradiente_fail;"
        "cmp %2, %%edx;"
        "jg Gradiente_fail;"
        "mov %%r12d, %%edx;"
        "add %%r14d, %%edx;"
        "cmp $0, %%edx;"
        "jl Gradiente_fail;"
        "cmp %3, %%edx;"
        "jg Gradiente_fail;"

        // IF CORRECTO (utilizamos EDX como auxiliar offset)
        // edx ya tiene (y+j) del anterior IF
        "imul %2, %%edx;"
        "add %%r13d, %%edx;"
        "add %%r15d, %%edx;" //EDX = offset
        //VAMOS A UTILIZAR RSI (la variable original de imagenO) para guardar también el offset.
        //Al finalizar, volveremos a ajustar RSI a su valor de imagenO
        //Por tanto RSI deberá tener dirImagenO + offset
        "movsxd %%edx, %%rdx;" //convertimos offset en 64
        "add %%rdx, %%rsi;" //sumamos dirImagenO + offset (64)
        "movzbl (%%rsi), %%esi;" //el contenido de dirImagenO+offset se guarda en RSI (32)
        "imul (%%r10), %%esi;" //el contenido de dirSobelX (64) se multiplica y guarda en RSI (32)
        "addl %%esi, %%r8d;" //Acumulamos esi (resultado final, 32) a R8 (gradX, 32)
        "mov %0, %%rsi;" // ESI = imagen (devolvemos su valor original)
        //Ahora toca el gradY
        //nos saltamos la conversión del offset
        "add %%rdx, %%rsi;" //sumamos dirImagenO + offset (64)
        "movzbl (%%rsi), %%esi;" //el contenido de dirImagenO+offset se guarda en RSI (32)
        "imul (%%r11), %%esi;" //el contenido de dirSobelY (64) se multiplica y guarda en RSI (32)
        "addl %%esi, %%r9d;" //Acumulamos esi (resultado final, 32) a R9 (gradY, 32)

        //IF FALLADO
        "Gradiente_fail:"
        "add $4, %%r10;"
        "add $4, %%r11;"
        "mov %0, %%rsi;" // ESI = imagen (devolvemos su valor original)

        "inc %%r15d;"
        "cmp $1, %%r15d;"
        "jle Gradientes_L4;"
        "inc %%r14d;"
        "cmp $1, %%r14d;"
        "jle Gradientes_L3;"

        //Utilizamos EDX para el offset
        "xor %%rdx, %%rdx;" // RDX = 0
        "mov %%r12d, %%edx;" // EDX = Y
        "imul %2, %%edx;" // EDX = Y * W
        "add %%r13d, %%edx;" // EDX = Y * W + X = offsetPixel2
        "movsxd %%edx, %%rdx;"
        "mov %%r8d, (%%rdi, %%rdx, 8);"
        "mov %%r9d, 4(%%rdi, %%rdx, 8);"

        "inc %%r13d;"
        "cmp %2, %%r13d;"
        "jl Gradientes_L2;"
        "inc %%r12d;"
        "cmp %3, %%r12d;"
        "jl Gradientes_L1;"
        :
        : "m" (imagen), "m" (gradientes), "m" (W), "m" (H), "m" (sobel_x), "m" (sobel_y)
        : "memory", "%rdx", "%rsi", "%rdi", "%r8", "%r9", "%r10", "%r11", "%r12", "%r13", "%r14", "%r15"

    );


}



void imageprocess::supresionNoMaximo(int * gradNorm, unsigned char * gradDir, unsigned char* imagenD, int W, int H)
{

    // %0 = gradNorm; %1 = gradDir; %2 = imagenD; %3 = W; %4 = H

    asm volatile(
        "mov %0, %%rsi;" //RSI = dirGradNorm * (32)
        "mov %1, %%rdi;" //RDI = dirGradDir * (8b)
        "mov %2, %%r8;" //R8 = dirImagenD * (8b)
        // Inicio bucle XY
        "mov $1, %%r9d;" // R9d = Y (reiniciar) L1
        "supresion_Y:"
        "mov $1, %%r10d;" // R10d = X (reiniciar) L2
        "supresion_X:"

        "mov %%r9d, %%r11d;"
        "imul %3, %%r11d;"
        "add %%r10d, %%r11d;" //R11d = offsetPixel (32)
        "movsxd %%r11d, %%r11;" //R11 = offsetPixel (64)

        // START --- 1º IF, EDX AUX, offset (64)
        "mov (%%rsi, %%r11, 4), %%edx;"
        "cmp $255, %%edx;"
        "jg supresion_I1N;"
        "mov %%edx, (%%r8, %%r11);"
        "jmp supresion_I1E;"

        "supresion_I1N:"
        "movb $255, (%%r8, %%r11);"

        "supresion_I1E:"
        // END --- 1º IF

        "xor %%r12, %%r12;" // R12 = vOffset1 (64)
        "xor %%r13, %%r13;" // R13 = vOffset2 (64)
        "mov %3, %%r14d;" // R14d = W (32)
        "movsxd %%r14d, %%r14;" // R14 = W (64)
        "xor %%rdx, %%rdx;"
        // START --- 2º IF-ENCADENADO, EDX AUX (8), offset (64)
        "mov (%%rdi, %%r11), %%dl;"
        "cmp $0, %%dl;"
        "je supresion_I2_1;"
        "cmp $1, %%dl;"
        "je supresion_I2_2;"
        "cmp $2, %%dl;"
        "je supresion_I2_3;"
        "cmp $3, %%dl;"
        "je supresion_I2_4;"
        "jmp supresion_I2E;"

        "supresion_I2_1:"
        "lea -1(%%r11), %%r12;"
        "lea 1(%%r11), %%r13;"
        "jmp supresion_I2E;"

        "supresion_I2_2:"
        "neg %%r14;"
        "lea -1(%%r11, %%r14), %%r12;"
        "neg %%r14;"
        "lea 1(%%r11, %%r14), %%r13;"
        "jmp supresion_I2E;"

        "supresion_I2_3:"
        "neg %%r14;"
        "lea (%%r11, %%r14), %%r12;"
        "neg %%r14;"
        "lea (%%r11, %%r14), %%r13;"
        "jmp supresion_I2E;"

        "supresion_I2_4:"
        "neg %%r14;"
        "lea 1(%%r11, %%r14), %%r12;"
        "neg %%r14;"
        "lea -1(%%r11, %%r14), %%r13;"
        "jmp supresion_I2E;"

        "supresion_I2E:"
        // END --- 2º IF-ENCADENADO

        "xor %%rdx, %%rdx;"
        "xor %%r15, %%r15;"
        // START --- 3º IF, EDX AUX (32), R15 AUX (32)
        "mov (%%rsi, %%r11, 4), %%r15d;"
        "mov (%%rsi, %%r12, 4), %%edx;"
        "cmp %%edx, %%r15d;"
        "jle supresion_I3;"
        "mov (%%rsi, %%r13, 4), %%edx;"
        "cmp %%edx, %%r15d;"
        "jl supresion_I3;"
        "jmp supresion_I3E;"

        "supresion_I3:"
        "movb $0, (%%r8, %%r11);"

        "supresion_I3E:"
        // END --- 3º IF



        // Fin bucle XY, utilizamos EDX como auxiliar
        "inc %%r10d;"
        "mov %3, %%edx;"
        "dec %%edx;"
        "cmp %%edx, %%r10d;"
        "jl supresion_X;"
        "inc %%r9d;"
        "mov %4, %%edx;"
        "dec %%edx;"
        "cmp %%edx, %%r9d;"
        "jl supresion_Y;"
        :
        : "m" (gradNorm), "m" (gradDir), "m" (imagenD), "m" (W), "m" (H)
        : "memory", "%rdx", "%rdi", "%rsi", "%r8", "%r9", "%r10", "%r11", "%r12", "%r13", "%r14", "%r15"
    );

}

void imageprocess::dobleUmbralizacion(unsigned char* imagenO, unsigned char* imagenD, unsigned char umbralMin, unsigned char umbralMax, int W, int H)
{

    // %0 = imagenO; %1 = imagenD; %2 = umbralMin; %3 = umbralMax; %4 = W; %5 = H

//Código en C:
//    unsigned char * dirImagenO = imagenO; // (8)
//    unsigned char * dirImagenD = imagenD; // (8)

//    for (int y = 1; y < H-1; y++){
//        for (int x = 1; x < W-1; x++){
//            int offsetPixel = y*W+x;
//            unsigned char * pO = dirImagenO;
//            pO += offsetPixel;
//            unsigned char * pD = dirImagenD;
//            pD += offsetPixel;
//            if (*pO >= umbralMax){
//                *pD = 255;
//            } else {
//                int esBorde = 0;
//                if (*pO >= umbralMin) {
//                    for (int j = -1; j <= 1 && esBorde == 0; j++){
//                        for (int i = -1; i <= 1 && esBorde == 0; i++){
//                            int vOffset = (y+j)*W + (x+i);
//                            pO = dirImagenO;
//                            pO += vOffset;
//                            pD = dirImagenD;
//                            pD += vOffset;
//                            if (*pO >= umbralMax || *pD == 255){
//                                esBorde = 1;
//                            }
//                        }
//                    }
//                }
//                pD = dirImagenD;
//                pD += offsetPixel;
//                if (esBorde == 1) {
//                    *pD = 255;
//                } else {
//                    *pD = 0;
//                }
//            }
//        }
//    }

    asm volatile(
        "mov %0, %%rsi;" // RSI * imagenO (64)->(8)
        "mov %1, %%rdi;" // RDI * imagenD (64)->(8)

        // Inicio del bucle XY
        "mov $1, %%r8d;" // R8d = Y (reiniciar) L1
        "dobleU_Y:"
        "mov $1, %%r9d;" // R9d = X (reiniciar) L2
        "dobleU_X:"

        "mov %%r8d, %%r10d;"
        "imul %4, %%r10d;"
        "add %%r9d, %%r10d;" // R10d = offsetPixel (32)
        "movsxd %%r10d, %%r10;" // R10 = offsetPixel (64)
        "mov %%rsi, %%r11;" // R11 = dirImagenO variable (64)
        "add %%r10, %%r11;" // dirImagenO + OffsetPixel
        "mov %%rdi, %%r12;" // R12 = dirImagenD variable (64)
        "add %%r10, %%r12;" // dirImagenD + OffsetPixel

        // START --- 1º IF, EDX AUX
        "xor %%rdx, %%rdx;"
        "mov (%%r11), %%dl;"
        "cmp %3, %%dl;"
        "jb dobleU_I1N;"
        "movb $255, (%%r12);"
        "jmp dobleU_I1E;"
        "dobleU_I1N:"

        "mov $0, %%r13b;" // R10b = esBorde (8)
        // START --- 2º IF, EDX AUX
        "mov (%%r11), %%dl;"
        "cmp %2, %%dl;"
        "jb dobleU_I2E;"

        // Inicio del bucle IJ
        "mov $-1, %%r14d;" // R14d = J (reiniciar) L3
        "dobleU_J:"
        "mov $-1, %%r15d;" // R15d = I (reiniciar) L4
        "dobleU_I:"

        "mov %%r8d, %%ecx;"
        "add %%r14d, %%ecx;"
        "imul %4, %%ecx;"
        "add %%r9d, %%ecx;"
        "add %%r15d, %%ecx;" // ECX = vOffset (32)
        "movsxd %%ecx, %%rcx;" // RCX = vOffset (64)
        "mov %%rsi, %%r11;"
        "add %%rcx, %%r11;" // dirImagenO + voffset (64)
        "mov %%rdi, %%r12;"
        "add %%rcx, %%r12;" // dirImagenD + voffset (64)

        // START --- 3º IF, EDX AUX
        "xor %%rdx, %%rdx;"
        "mov (%%r11), %%dl;"
        "cmp %3, %%dl;"
        "jb dobleU_I3E;"
        "mov $1, %%r13b;"
        "dobleU_I3E:"
        // FIN --- 3º IF

        // Fin del bucle IJ, utilizamos EDX como auxiliar
        "inc %%r15d;"
        "cmp $1, %%r15d;"
        "jg dobleU_I_fail;"
        "cmp $0, %%r13b;"
        "je dobleU_I;"
        "dobleU_I_fail:"
        "inc %%r14d;"
        "cmp $1, %%r14d;"
        "jg dobleU_J_fail;"
        "cmp $0, %%r13b;"
        "je dobleU_J;"
        "dobleU_J_fail:"

        "dobleU_I2E:"
        // FIN --- 2º IF

        "mov %%rdi, %%r12;" //
        "add %%r10, %%r12;" // dirImagenD + OffsetPixel
        // START --- 4º IF, EDX AUX
        "cmp $1, %%r13b;"
        "jne dobleU_I4N;"
        "movb $255, (%%r12);"
        "jmp dobleU_I4E;"
        "dobleU_I4N:"
        "movb $0, (%%r12);"
        "dobleU_I4E:"
        // FIN --- 4º IF

        "dobleU_I1E:"
        // FIN --- 1º IF

        // Fin bucle XY, utilizamos EDX como auxiliar
        "inc %%r9d;"
        "mov %4, %%edx;"
        "dec %%edx;"
        "cmp %%edx, %%r9d;"
        "jl dobleU_X;"
        "inc %%r8d;"
        "mov %5, %%edx;"
        "dec %%edx;"
        "cmp %%edx, %%r8d;"
        "jl dobleU_Y;"
        :
        : "m" (imagenO), "m" (imagenD), "m" (umbralMin), "m" (umbralMax), "m" (W), "m" (H)
        : "memory", "%rcx", "%rdx", "%rdi", "%rsi", "%r8", "%r9", "%r10", "%r11", "%r12", "%r13", "%r14", "%r15"

    );

}


int imageprocess::detectaContorno(unsigned char * imagen, unsigned char * visitados, int ini_x, int ini_y, Point direcciones[8], Point contorno[5000], int W, int H)
{
    int nPuntos = 0;

    // %0 = nPuntos; %1 = imagen; %2 = visitados; %3 = ini_x; %4 = ini_y; %5 = direcciones; %6 = contorno; %7 = W; %8 = H

// Código en C:
//    unsigned char * dirImagen = imagen;
//    unsigned char * dirVisitados = visitados;
//    Point * dirContorno = contorno;
//    Point * dirDirecciones = direcciones;

//    int sig_x = ini_x;
//    int sig_y = ini_y;
//    int iContorno = 0;
//    bool hayPunto = true;

//    while(hayPunto && iContorno<4999){
//        Point * p = dirContorno;
//        p += iContorno;
//        p->x = sig_x;
//        p->y = sig_y;
//        int offsetPixel = sig_y*W + sig_x;
//        unsigned char * p2 = dirVisitados;
//        p2 += offsetPixel;
//        *p2 = 255;
//        iContorno++;

//        int iDir = 0;
//        hayPunto = false;
//        while(iDir<8 && !hayPunto){
//            Point * p3 = dirDirecciones;
//            p3 += iDir;

//            int vec_x = sig_x + p3->x;
//            int vec_y = sig_y + p3->y;
//            if (vec_x>=0 && vec_x<W && vec_y>=0 && vec_y<H){
//                int offset_vec = vec_y*W + vec_x;
//                unsigned char * p4 = dirVisitados;
//                p4 += offset_vec;
//                unsigned char * p5 = dirImagen;
//                p5 += offset_vec;
//                if (*p4 == 0 && *p5 == 255){
//                    hayPunto = true;
//                    sig_x = vec_x;
//                    sig_y = vec_y;
//                }
//            }
//            iDir++;
//        }
//    }
//    nPuntos = iContorno;

asm volatile(
    "mov %3, %%r8d;" // R8d = sig_x (32)
    "mov %4, %%r9d;" // R9d = sig_y (32)
    "mov $0, %%r10d;" // R10d = iContorno (32)
    "mov $1, %%r11b;" // R11b = hayPunto (8)

    //Inicio del while 1
    "contorno_W1:"
    "cmp $1, %%r11b;"
    "jne contorno_W1E;"
    "cmp $4999, %%r10d;"
    "jae contorno_W1E;"

    // Usamos EDX, ESI como auxiliar
    "mov %6, %%rsi;"
    "mov %%r10d, %%edx;"
    "movsxd %%edx, %%rdx;" //iContorno (64)
    "imul $8, %%rdx;" //iContorno * 8
    "add %%rdx, %%rsi;" // RSI = dirContorno + iContorno*8 (64)
    "mov %%r8d, (%%rsi);"
    "mov %%r9d, 4(%%rsi);"
    // Calculamos offsetPixel (al ser de carácter temporal, utilizamos EDX)
    "xor %%rdx, %%rdx;"
    "mov %%r9d, %%edx;"
    "imul %7, %%edx;"
    "add %%r8d, %%edx;"
    "movsxd %%edx, %%rdx;" // RDX = offsetPixel (64)
    //Establecemos y ponemos *p2 a 255
    "add %2, %%rdx;"
    "movb $255, (%%rdx);"
    "inc %%r10d;"

    "xor %%r12, %%r12;" // R12 = iDir (64)
    "mov $0, %%r11b;"
    //Inicio del While 2
    "contorno_W2:"
    "cmp $8, %%r12d;"
    "jae contorno_W2E;"
    "cmp $0, %%r11b;"
    "jne contorno_W2E;"
    // Calculamos vec_x e y
    // Primero antes, calculamos la dirección. Utilizamos EDX temporalmente.
    "xor %%rdx, %%rdx;"
    "mov %%r12, %%rdx;"
    "imul $8, %%rdx;"
    "add %5, %%rdx;"
    "mov (%%rdx), %%r13d;" // R13d = vec_x (32)
    "add %%r8d, %%r13d;"
    "mov 4(%%rdx), %%r14d;" // R14d = vec_y (32)
    "add %%r9d, %%r14d;"

    // START --- 1º IF
    "cmp $0, %%r13d;"
    "jb contorno_I1E;"
    "cmp %7, %%r13d;"
    "jae contorno_I1E;"
    "cmp $0, %%r14d;"
    "jb contorno_I1E;"
    "cmp %8, %%r14d;"
    "jae contorno_I1E;"
    //Calculamos offset_vec, utilizando ESI como auxiliar
    "xor %%rsi, %%rsi;"
    "mov %%r14d, %%esi;"
    "imul %7, %%esi;"
    "add %%r13d, %%esi;" // ESI = offset_vec (32)
    "movsxd %%esi, %%rsi;" // RSI = offset_vec (64)
    // START --- 2º IF, RDI, EDX, ECX AUX
    //Calculamos 1º cond
    "xor %%rdi, %%rdi;"
    "xor %%rdx, %%rdx;"
    "mov %2, %%rdi;"
    "add %%rsi, %%rdi;"
    "mov (%%rdi), %%dl;"
    //Calculamos 2º cod
    "xor %%rcx, %%rcx;"
    "xor %%rdi, %%rdi;"
    "mov %1, %%rdi;"
    "add %%rsi, %%rdi;"
    "mov (%%rdi), %%cl;"
    //Comprobamos
    "cmp $0, %%dl;"
    "jne contorno_I2E;"
    "cmp $255, %%cl;"
    "jne contorno_I2E;"

    "mov $1, %%r11b;"
    "mov %%r13d, %%r8d;"
    "mov %%r14d, %%r9d;"

    "contorno_I2E:"
    // FIN --- 2º IF

    "contorno_I1E:"
    // FIN --- 1º IF

    "inc %%r12;"

    "jmp contorno_W2;"
    "contorno_W2E:"
    //Fin del While 2

    "jmp contorno_W1;"
    "contorno_W1E:"
    // Fin del While 1

    "mov %%r10d, %0;"

    : "=m" (nPuntos)
    : "m" (imagen), "m" (visitados), "m" (ini_x), "m" (ini_y), "m" (direcciones), "m" (contorno), "m" (W), "m" (H)
    : "memory", "%rcx", "%rdx", "%rdi", "%rsi", "%r8", "%r9", "%r10", "%r11", "%r12", "%r13", "%r14"

);


    return nPuntos;
}


int imageprocess::contornoAPoligono(Point contorno[5000], int nPuntos, Point poligono[5000], float distancia)
{
    int nP = 0;

    // %rsi = contour; %eax = nPoints; %rdi = polygon; %edx = nP

    // NOTAS IMPORTANTES:
    // PARA USAR LA FPU, es necesario utilizar ELEMENTOS EN MEMORIA. NO SIRVEN REGISTROS.
    asm volatile(
        "mov $0, %%edx;" //nP = 0
        "flds %4;" //distancia en %st(0) para implementación con FPU
        "movss %4, %%xmm0;" //distancia en %xmm0 para implementación con SSE
        "call contornoAPoligonoRecursivo;"
        "fstp %%st(0);"
        "jmp finContornoAPoligono;"

        "contornoAPoligonoRecursivo:"
        // IMPLEMENTACIÓN DEL PROCEDIMIENTO contornoAPoligonoRecursivo

        "mov %%eax, %%r8d;" // iF = R8d (32)
        "movsxd %%r8d, %%r8;"
        "dec %%r8d;"
        //Calcular A, B y C
        "flds 4(%%rsi, %%r8, 8);"
        "fsubs 4(%%rsi);" // a
        "flds (%%rsi);"
        "fsubs 4(%%rsi, %%r8, 8);" // b a
        "fld %%st(0);" // b b a
        "fmul 4(%%rsi);" // b*dir b a
        "fld %%st(2);" // a b*dir b a
        "fchs;" // -a b*dir b a
        "fmul (%%rsi);" // -a*dir b*dir b a
        "fsub %%st(1), %%st(0);" // c b*dir b a
        "fxch %%st(1);" // b*dir c b a
        "fstp %%st(0);" // c b a
        // Calcular norm
        "fld %%st(2);" // a c b a
        "fmul %%st(0);" // a*a c b a
        "fld %%st(2);" // b a*a c b a
        "fmul %%st(0);" // b*b a*a c b a
        "faddp %%st(0), %%st(1);" // a*a+b*b c b a
        "fsqrt;" // norm c b a
        // Actualizar valores de A, B y C
        "fdiv %%st(0), %%st(1);"
        "fdiv %%st(0), %%st(2);"
        "fdivp %%st(0), %%st(3);" // c b a

        "xor %%r9, %%r9;" // dMax = R9d (32)
        "xor %%r10, %%r10;" // iCorte = R10d (32)

        // Inicio bucle i
        "mov $0, %%r11d;" // R11d = i (reiniciar) L1
        "poligono_i:"

        // Convertimos i (R11) de 32 a 64 (R12d)
        "movsxd %%r11d, %%r12;"
        // Calcular dAct
        "fld %%st(2);" // a c b a
        "fmul (%%rsi, %%r12, 8);" // a*dir c b a
        "fld %%st(2);" // b a*dir c b a
        "fmul 4(%%rsi, %%r12, 8);" // b*dir a*dir c b a
        "faddp %%st(0), %%st(1);" // a*dir+b*dir c b a
        "fadd %%st(1), %%st(0);" // a*dir+b*dir+c c b a
        "fabs;" // |a*dir+b*dir+c| c b a

        // Fin bucle i, utilizamos EDX como auxiliar
        "inc %%r11d;"
        "cmp %%r8d, %%r11d;"
        "jl poligono_i;"




        "ret;"

        "finContornoAPoligono:"

        : "=d" (nP)
        : "S" (contorno), "a" (nPuntos), "D" (poligono), "m" (distancia)
        : "%xmm0", "memory", "%r8", "%r9", "%r10", "%r11"
    );


    return nP;

}







