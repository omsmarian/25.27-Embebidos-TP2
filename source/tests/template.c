#include <stdio.h>
#include <string.h>

#include "modulo_a_testear.h"

int pruebas_totales = 0;
int pruebas_pass = 0;
int pruebas_fail = 0;

void check_string_iguales(char * esperado, char * actual);
void check_punteros_iguales(char * esperado, char * actual);

int main(void)
{
   printf("Corriendo pruebas en modulo...");

   check_punteros_iguales(NULL, func(0));

   check_string_iguales("0", func(0));

   printf("\n%d pruebas realizadas: %d PASS, %d FAIL\n",pruebas_totales,pruebas_pass,pruebas_fail);

   return 0;
}

void check_string_iguales(char * esperado, char * actual)
{
   pruebas_totales++;

   if(strcmp(esperado,actual)==0)
   {
      pruebas_pass++;
   }
   else
   {
      pruebas_fail++;
      printf("\nFAIL - esperado: %s - actual: %s", esperado, actual);
   }
}

void check_punteros_iguales(char * esperado, char * actual)
{
   pruebas_totales++;

   if(esperado == actual)
   {
      pruebas_pass++;
   }
   else
   {
      pruebas_fail++;
      printf("\nFAIL - esperado: %p - actual: %p", esperado, actual);
   }
}

size_t func (size_t)
{
	funcion_a_testear();

	return 0;
}
