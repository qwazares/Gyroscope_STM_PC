#include "FFT.h"

void bitrev(Tablica *dane) //funkcja inwersji bitow
{
   int i2,j,k,i;
   double temp;
   i2 = dane->rozmiar >> 1;
   j = 0;

   for (i = 0; i < dane->rozmiar-1 ; i++)
   {
      if (i < j)
    	{
    		temp = dane->real[i];
    		dane->real[i]=dane->real[j];
    		dane->real[j]=temp;
    		
    		temp = dane->im[i];
    		dane->im[i]=dane->im[j];
    		dane->im[j]=temp;
        }

      k = i2;

      while (k <= j) 
	  {
         j -= k;
         k >>= 1;
      }

      j += k;
   }
}

void FFT(Tablica *dane,char IFFT) 
{
    unsigned int n=0, nspan, span, submatrix, node, i;
    double temp, primitive_root, angle, realtwiddle, imtwiddle;

     

    for(span=dane->rozmiar>>1; span; span>>=1)      // petla g³owna FFT
    {
       primitive_root = MINPI/span;
      
       for(submatrix=0; submatrix<(dane->rozmiar>>1)/span; submatrix++)
       {
          for(node=0; node<span; node++)
          {
            nspan = n+span;
            temp = dane->real[n] + dane->real[nspan];       
            dane->real[nspan] = dane->real[n]-dane->real[nspan];
            dane->real[n] = temp;
            temp = dane->im[n] + dane->im[nspan];
            dane->im[nspan] = dane->im[n] - dane->im[nspan];
            dane->im[n] = temp;
           
            angle = primitive_root * node;      // rotacja
            realtwiddle = cos(angle);
            imtwiddle = IFFT * sin(angle);
            temp = realtwiddle * dane->real[nspan] - imtwiddle * dane->im[nspan];
            dane->im[nspan] = realtwiddle * dane->im[nspan] + imtwiddle * dane->real[nspan];
            dane->real[nspan] = temp;
           
            n++;   
           
          } 
        
          n = (n+span) & (dane->rozmiar-1);   //pomija nieparzyste bloki
       
        } 
       
     } // koniec pêtli g³ownej FFT
     
     if(IFFT == -1)		//dla trybu IFFT podzielenie wartosci przez ilosc probek
     	for(i = 0; i < dane->rozmiar; i++)
     	{
     		dane->im[i] /= dane->rozmiar;
     		dane->real[i] /= dane->rozmiar;
     	}

} 

