//---------------------------------------------------------------------------

#include <vcl.h>
#pragma hdrstop

#include "Unit1.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma link "PERFGRAP"
#pragma resource "*.dfm"
TForm1 *Form1;

#define RS_speed 230400
#define max_danych 4096

//-------------------BAJT STARTU -------------------------------
#define START_BIT 7

#define BIT_X_L 0
#define BIT_X_H 1
#define BIT_Y_L 2
#define BIT_Y_H 3
#define BIT_Z_L 4
#define BIT_Z_H 5

//------------------ Bajt konfiguracyjny -------------------------
#define TRANSMITION_BIT 0
#define FULLSCALE_BIT_1 1
#define FULLSCALE_BIT_2 2
#define DATARATE_BIT_1 3
#define DATARATE_BIT_2 4

#define FULLSCALE_MASK (1<<FULLSCALE_BIT_1 | 1<<FULLSCALE_BIT_2)
#define FULLSCALE_BIT_250 0
#define FULLSCALE_BIT_500 (1<<FULLSCALE_BIT_1)
#define FULLSCALE_BIT_2000 (1<<FULLSCALE_BIT_2)

#define DATARATE_MASK (1<<DATARATE_BIT_1 | 1<<DATARATE_BIT_2)
#define DATARATE_BIT_95		0
#define DATARATE_BIT_190	(1<<DATARATE_BIT_1)
#define DATARATE_BIT_380	(1<<DATARATE_BIT_2)
#define DATARATE_BIT_760	(1<<DATARATE_BIT_1 | 1<<DATARATE_BIT_2)

//------------ parametry okna -----------------
#define left_label 42
#define right_label 20
#define width_between 15

#define top_label 78
#define bottom_label 60
#define height_between 1

HANDLE hCommDev;
DCB dcb;
LPCTSTR lpFileName;
bool b_otwarty;
bool b_odbior;
unsigned long ile_RS;
char COMchar[7];
//-------------- dane --------------------------------
BYTE data[10];           //dane odbierane
short axes[3];

BYTE RS_data = FULLSCALE_BIT_2000 | DATARATE_BIT_760;           //wysy³anie

short *data_x, *data_y, *data_z;
short *buffor_x, *buffor_y, *buffor_z;

double *fft_x, *fft_y, *fft_z;
double *buffer_fft;
//-------------- dane konfiguracyjne -----------------

int ilosc_danych;
int dzielnik_fft;
int odstepy;
int param;
int mala_skala;

int gyro_scale = 2000;
int gyro_speed = 760;


//---------------------------------------------------------------------------
__fastcall TForm1::TForm1(TComponent* Owner)
        : TForm(Owner)
{
        b_otwarty = false;
        b_odbior = false;
        ilosc_danych = 1024;
        dzielnik_fft = 2;
        odstepy = 40;
        param = 1;
        mala_skala = 1;

        Form1->DoubleBuffered = true;

        data_x = (short*) calloc(max_danych, sizeof(short));
        data_y = (short*) calloc(max_danych, sizeof(short));
        data_z = (short*) calloc(max_danych, sizeof(short));
        buffor_x = (short*) calloc(max_danych, sizeof(short));
        buffor_y = (short*) calloc(max_danych, sizeof(short));
        buffor_z = (short*) calloc(max_danych, sizeof(short));

        fft_x = (double*) calloc(max_danych, sizeof(double));
        fft_y = (double*) calloc(max_danych, sizeof(double));
        fft_z = (double*) calloc(max_danych, sizeof(double));
        buffer_fft = (double*) calloc(max_danych, sizeof(double));

        //------------ rozdzielenie wykresów -------------------
        Image7->Top = Image2->Top -2;
        Image7->Left = 0;
        Image7->Height = 3;
        Image7->Width = Form1->Width;

        Image7->Canvas->Pen->Width = 3;
        Image7->Canvas->MoveTo(0,1);
        Image7->Canvas->LineTo(Image7->Width,1);

        Image8->Top = Image3->Top -2;
        Image8->Left = 0;
        Image8->Height = 3;
        Image8->Width = Form1->Width;

        Image8->Canvas->Pen->Width = 3;
        Image8->Canvas->MoveTo(0,1);
        Image8->Canvas->LineTo(Image8->Width,1);

        Label4->Top = Image1->Top + Image1->Height/2 - Label4->Height/2;
        Label5->Top = Image2->Top + Image2->Height/2 - Label5->Height/2;
        Label6->Top = Image3->Top + Image3->Height/2 - Label6->Height/2;
        

        Czysc_rys();
        zmiana_zakresu();

        //Skanuj1Click(Skanuj1);


}
//---------------------------------------------------------------------------

void __fastcall TForm1::Open_CloseClick(TObject *Sender)
{
   if(b_otwarty == false)
   {
       strcpy(COMchar,ComPorts->Text.c_str());
       lpFileName = COMchar;
       hCommDev = CreateFile(lpFileName, GENERIC_WRITE |     // OTWARCIE PORTU
       GENERIC_READ, 0, NULL,
       OPEN_EXISTING, 0, NULL);

       if(hCommDev != INVALID_HANDLE_VALUE && hCommDev != NULL)
       {
           dcb.DCBlength = sizeof(dcb); // aktualny rozmiar
                                        // struktury DCB
           GetCommState(hCommDev, &dcb); // udostêpnienie aktualnych
                                        // parametrów DCB
           dcb.BaudRate = RS_speed; // prêdkoœæ transmisji
           dcb.fParity = FALSE; // sprawdzanie parzystoœci
                        //dcb.Parity = NOPARITY; // ustawienie parzystoœci
           dcb.StopBits = ONESTOPBIT; // bity stopu
           dcb.ByteSize = 8; // bity danych
           dcb.fDtrControl = 1; // np. kontrola linii DTR
           SetCommState(hCommDev, &dcb); // reinicjalizacja DCB

           b_otwarty = true;
           Open_Close->Caption = "Zamknij port";

           B_odbierz->Enabled = true;
           SendConf->Enabled = true;
       }
    }
    else
    {
        B_odbierz->Caption = "Odbierz";
        RS_data &= (0xFF ^ 1<<TRANSMITION_BIT);
        WriteFile(hCommDev, &RS_data, 1,&ile_RS , 0);
        CloseHandle(hCommDev);
        Timer1->Enabled = false;

        b_otwarty = false;
        b_odbior = false;

        Open_Close->Caption = "Otwórz port";
        B_odbierz->Enabled = false;
        SendConf->Enabled = false;
        for(int i=0; i<max_danych; i++)
        {
            data_x[i] = 0;
            data_y[i] = 0;
            data_z[i] = 0;
        }

        Czysc_rys();
    }
}
//---------------------------------------------------------------------------
void __fastcall TForm1::B_odbierzClick(TObject *Sender)
{
        b_odbior = !b_odbior;
        //Timer1->Enabled = !Timer1->Enabled;

        if(b_odbior)
        {
        B_odbierz->Caption = "Zatrzymaj";
        RS_data |= 1<<TRANSMITION_BIT;
        WriteFile(hCommDev, &RS_data, 1,&ile_RS , 0);
        Timer1->Enabled = true;
        }
 else
       {
        B_odbierz->Caption = "Odbierz";
        RS_data &= (0xFF ^ 1<<TRANSMITION_BIT);
        WriteFile(hCommDev, &RS_data, 1,&ile_RS , 0);
        }


}
//---------------------------------------------------------------------------
void __fastcall TForm1::Timer1Timer(TObject *Sender)
{

COMSTAT Stat;
DWORD Errors;
DWORD nNumberOfBytesToRead;

int ilosc;
bool kolejka = false;

                ClearCommError(hCommDev, &Errors, &Stat);
                while(Stat.cbInQue >= 8)
                {
                    ReadFile(hCommDev, data, 1,&ile_RS , 0);    //sprawdzenie bajtu startu
                    if(data[0] & 1<<START_BIT)
                    {
                        ReadFile(hCommDev, &data[1], 7,&ile_RS , 0);     //odczyt danych

                        if(data[1] & 1<<TRANSMITION_BIT)
                        {
                        data[2] |=  (data[0] & 1<<BIT_X_L)>>BIT_X_L<<7;           //ustawienie najstarszego bitu
                        data[3] |=  (data[0] & 1<<BIT_X_H)>>BIT_X_H<<7;
                        data[4] |=  (data[0] & 1<<BIT_Y_L)>>BIT_Y_L<<7;
                        data[5] |=  (data[0] & 1<<BIT_Y_H)>>BIT_Y_H<<7;
                        data[6] |=  (data[0] & 1<<BIT_Z_L)>>BIT_Z_L<<7;
                        data[7] |=  (data[0] & 1<<BIT_Z_H)>>BIT_Z_H<<7;

                        axes[0] = data[2] + (data[3]<<8);
                        axes[1] = data[4] + (data[5]<<8);
                        axes[2] = data[6] + (data[7]<<8);

                        memcpy(&buffor_x[1],data_x,sizeof(short)*(max_danych - 1));  //dane buforowane
                        memcpy(&buffor_y[1],data_y,sizeof(short)*(max_danych - 1));
                        memcpy(&buffor_z[1],data_z,sizeof(short)*(max_danych - 1));

                        buffor_x[0] = axes[0];
                        buffor_y[0] = axes[1];
                        buffor_z[0] = axes[2];

                        memcpy(data_x,buffor_x,sizeof(short)*max_danych);
                        memcpy(data_y,buffor_y,sizeof(short)*max_danych);
                        memcpy(data_z,buffor_z,sizeof(short)*max_danych);

                        kolejka = true;
                        }

                        show_config();

                        if(!(data[1] & 1<<TRANSMITION_BIT))
                        {
                            Timer1->Enabled = false;
                            kolejka = true;
                        }
                    }
                    ClearCommError(hCommDev, &Errors, &Stat);

                }

                if(kolejka)
                {
                    FFT_dane();
                    Rys_dane();
                }


                return;


}
//---------------------------------------------------------------------------


void __fastcall TForm1::FormClose(TObject *Sender, TCloseAction &Action)
{
        if(b_otwarty)
             if(Application->MessageBox("Zamkn¹æ program?", "Zamykanie programu", MB_YESNO | MB_ICONQUESTION) == ID_YES)
             {
                  RS_data &= (0xFF ^ 1<<TRANSMITION_BIT);
                  WriteFile(hCommDev, &RS_data, 1,&ile_RS , 0);


                  free(data_x);
                  free(data_y);
                  free(data_z);

                  free(buffor_x);
                  free(buffor_y);
                  free(buffor_z);

                  free(fft_x);
                  free(fft_y);
                  free(fft_z);
                  free(buffer_fft);

                  Action = caFree;

                  CloseHandle(hCommDev);
             }
             else
                  Action = caNone;
}

//---------------------------------------------------------------------------

void __fastcall TForm1::FFT_dane()
{
        Tablica data;

        for(int i = 0; i < ilosc_danych; i++)
        {
                buffer_fft[i] = 0;
                fft_x[i] = (double) data_x[i]/32767;
                fft_y[i] = (double) data_y[i]/32767;
                fft_z[i] = (double) data_z[i]/32767;
        }

        /********** os x **********************/
        data.real = fft_x;
        data.im = buffer_fft;
        data.rozmiar = ilosc_danych;

        FFT(&data,1);
        bitrev(&data);

        for(int i = 0; i < ilosc_danych; i++)
        {
                data.real[i] = sqrt(data.real[i]*data.real[i] + data.im[i]*data.im[i]);
                data.im[i] = 0;
        }
        /********** os y **********************/

        data.real = fft_y;

        FFT(&data,1);
        bitrev(&data);

        for(int i = 0; i < ilosc_danych; i++)
        {
                data.real[i] = sqrt(data.real[i]*data.real[i] + data.im[i]*data.im[i]);
                data.im[i] = 0;
        }
        /********** os z **********************/

        data.real = fft_z;

        FFT(&data,1);
        bitrev(&data);

        for(int i = 0; i < ilosc_danych; i++)
        {
                data.real[i] = sqrt(data.real[i]*data.real[i] + data.im[i]*data.im[i]);
                data.im[i] = 0;
        }

}

//-----------------------------------------------------------------------------------------------------------
void __fastcall TForm1::bitrev(Tablica *dane) //funkcja inwersji bitow
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

//-------------------------------------------------------------------------------------------------
void __fastcall TForm1::FFT(Tablica *dane,short IFFT)
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

//----------------------------------------------------------------------------
void __fastcall TForm1::Czysc_rys()
{
        float rozdzielczosc;

        rozdzielczosc = (float)gyro_speed/ilosc_danych;

        Image1->Canvas->Pen->Color = clBlack;
        Image1->Canvas->Pen->Width = 1;

        Image2->Canvas->Pen->Color = clBlack;
        Image2->Canvas->Pen->Width = 1;

        Image3->Canvas->Pen->Color = clBlack;
        Image3->Canvas->Pen->Width = 1;

        //----- rysowanie ramki --------------------
        Image1->Canvas->Rectangle(0,0,Image1->Width,Image1->Height);
        Image2->Canvas->Rectangle(0,0,Image1->Width,Image1->Height);
        Image3->Canvas->Rectangle(0,0,Image1->Width,Image1->Height);
        Image4->Canvas->Rectangle(0,0,Image4->Width,Image4->Height);
        Image5->Canvas->Rectangle(0,0,Image5->Width,Image5->Height);
        Image6->Canvas->Rectangle(0,0,Image6->Width,Image6->Height);

        Form1->Canvas->MoveTo(0,Image2->Top);
        Form1->Canvas->LineTo(Form1->Width,Image2->Top);

        //------- rysowanie osi ----------------
        Image1->Canvas->Pen->Width = 1;
        Image2->Canvas->Pen->Width = 1;
        Image3->Canvas->Pen->Width = 1;

        Image1->Canvas->MoveTo(0,Image1->Height/2);
        Image1->Canvas->LineTo(Image1->Width,Image1->Height/2);

        Image2->Canvas->MoveTo(0,Image1->Height/2);
        Image2->Canvas->LineTo(Image1->Width,Image1->Height/2);


        Image3->Canvas->MoveTo(0,Image1->Height/2);
        Image3->Canvas->LineTo(Image1->Width,Image1->Height/2);

        Image4->Canvas->Rectangle(0,0,Image4->Width,Image4->Height);
        Image5->Canvas->Rectangle(0,0,Image5->Width,Image5->Height);
        Image6->Canvas->Rectangle(0,0,Image6->Width,Image6->Height);



        if(Siatka1->Checked)
        {

            //------ siatka dla osi czasu



            Label8->Visible = true;
            Label8->Left = Image1->Left + Image1->Width - (float)Image1->Width/ilosc_danych*gyro_speed*param/mala_skala - Label8->Width/2;

            for(float i = param; i <= ilosc_danych*mala_skala/gyro_speed; i += param)
            {
                Image1->Canvas->MoveTo(Image1->Width-(float)Image1->Width/ilosc_danych*gyro_speed*i/mala_skala,0);
                Image1->Canvas->LineTo(Image1->Width-(float)Image1->Width/ilosc_danych*gyro_speed*i/mala_skala,Image1->Height);
                Image2->Canvas->MoveTo(Image2->Width-(float)Image2->Width/ilosc_danych*gyro_speed*i/mala_skala,0);
                Image2->Canvas->LineTo(Image2->Width-(float)Image2->Width/ilosc_danych*gyro_speed*i/mala_skala,Image2->Height);
                Image3->Canvas->MoveTo(Image3->Width-(float)Image3->Width/ilosc_danych*gyro_speed*i/mala_skala,0);
                Image3->Canvas->LineTo(Image3->Width-(float)Image3->Width/ilosc_danych*gyro_speed*i/mala_skala,Image3->Height);
            }

            //------ siatka dla osi czêstotliwosci ---------------------



            Label9->Caption = IntToStr(odstepy) + " Hz";
            Label9->Visible = true;
            Label9->Left = Image4->Left + (float)Image4->Width/ilosc_danych*dzielnik_fft*odstepy/rozdzielczosc - Label9->Width/2;

            for(int i = 1; i <=((float)ilosc_danych/dzielnik_fft/odstepy*rozdzielczosc) ; i++)
            {
                Image4->Canvas->MoveTo(((float)Image4->Width*i)/ilosc_danych*dzielnik_fft*odstepy/rozdzielczosc,0);
                Image4->Canvas->LineTo(((float)Image4->Width*i)/ilosc_danych*dzielnik_fft*odstepy/rozdzielczosc,Image4->Height);
                Image5->Canvas->MoveTo(((float)Image5->Width*i)/ilosc_danych*dzielnik_fft*odstepy/rozdzielczosc,0);
                Image5->Canvas->LineTo(((float)Image5->Width*i)/ilosc_danych*dzielnik_fft*odstepy/rozdzielczosc,Image5->Height);
                Image6->Canvas->MoveTo(((float)Image6->Width*i)/ilosc_danych*dzielnik_fft*odstepy/rozdzielczosc,0);
                Image6->Canvas->LineTo(((float)Image6->Width*i)/ilosc_danych*dzielnik_fft*odstepy/rozdzielczosc,Image6->Height);
            }
        }
        else
        {
            Label8->Visible = false;
            Label9->Visible = false;
        }

        EX->Text = FloatToStr((float)((int)((float)data_x[0]/32767*gyro_scale*100))/100);
        EY->Text = FloatToStr((float)((int)((float)data_y[0]/32767*gyro_scale*100))/100);
        EZ->Text = FloatToStr((float)((int)((float)data_z[0]/32767*gyro_scale*100))/100);

        
}

//---------------------------------------------------------------------------


void __fastcall TForm1::Rys_dane()
{

        Czysc_rys();

        Image1->Canvas->Pen->Color = clBlack;
        Image1->Canvas->Pen->Width = 1;

        Image2->Canvas->Pen->Color = clBlack;
        Image2->Canvas->Pen->Width = 1;

        Image3->Canvas->Pen->Color = clBlack;
        Image3->Canvas->Pen->Width = 1;

        Image1->Canvas->MoveTo(Image1->Width,Image1->Height-((float)Image1->Height/0xffff)*(32767+data_x[0]));
        Image2->Canvas->MoveTo(Image2->Width,Image2->Height-((float)Image2->Height/0xffff)*(32767+data_y[0]));
        Image3->Canvas->MoveTo(Image3->Width,Image3->Height-((float)Image3->Height/0xffff)*(32767+data_z[0]));

        Image4->Canvas->MoveTo(0,Image4->Height-((float)Image4->Height/(ilosc_danych/2))*(fft_x[0]));
        Image5->Canvas->MoveTo(0,Image5->Height-((float)Image5->Height/(ilosc_danych/2))*(fft_y[0]));
        Image6->Canvas->MoveTo(0,Image6->Height-((float)Image6->Height/(ilosc_danych/2))*(fft_z[0]));

        for(int i = 1; i < ilosc_danych; i++)
        {
                Image1->Canvas->LineTo(Image1->Width - ((float)Image1->Width*i)/ilosc_danych,Image1->Height-((float)Image1->Height/0xffff)*(32767+data_x[i]));
                Image2->Canvas->LineTo(Image2->Width - ((float)Image2->Width*i)/ilosc_danych,Image2->Height-((float)Image2->Height/0xffff)*(32767+data_y[i]));
                Image3->Canvas->LineTo(Image3->Width - ((float)Image3->Width*i)/ilosc_danych,Image3->Height-((float)Image3->Height/0xffff)*(32767+data_z[i]));
        }



        for(int i = 1; i <= ilosc_danych/dzielnik_fft; i++)
        {
                Image4->Canvas->LineTo(((float)Image4->Width*i)/(ilosc_danych/dzielnik_fft),Image4->Height-((float)Image4->Height/(ilosc_danych/2))*(fft_x[i]));
                Image5->Canvas->LineTo(((float)Image5->Width*i)/(ilosc_danych/dzielnik_fft),Image5->Height-((float)Image5->Height/(ilosc_danych/2))*(fft_y[i]));
                Image6->Canvas->LineTo(((float)Image6->Width*i)/(ilosc_danych/dzielnik_fft),Image6->Height-((float)Image6->Height/(ilosc_danych/2))*(fft_z[i]));
        }


}

//------------------------------------------------------------------------------------------------------------------------------------------

void __fastcall TForm1::N5121Click(TObject *Sender)
{
        N5121->Checked = true;

        ilosc_danych = 512;

        if(gyro_speed == 760)
            mala_skala = 2;
        else
            mala_skala = 1;
        param = 1;
        Label8->Caption = FloatToStr((float)param/mala_skala) + " sek";

        FFT_dane();
        Rys_dane();
}
//---------------------------------------------------------------------------

void __fastcall TForm1::N10241Click(TObject *Sender)
{
     N10241->Checked = true;

     ilosc_danych = 1024;

     param = 1;
     mala_skala = 1;
     Label8->Caption = IntToStr(param) + " sek";
     FFT_dane();
     Rys_dane();
}
//---------------------------------------------------------------------------

void __fastcall TForm1::N20481Click(TObject *Sender)
{
    N20481->Checked = true;

    ilosc_danych = 2048;
    param = 1;
    mala_skala = 1;
     Label8->Caption = IntToStr(param) + " sek";
    FFT_dane();
    Rys_dane();
}
//---------------------------------------------------------------------------

void __fastcall TForm1::N40961Click(TObject *Sender)
{
    N40961->Checked = true;

    ilosc_danych = 4096;

    if(gyro_speed == 95)
        param = 2;
    else
        param = 1;
    mala_skala = 1;
    Label8->Caption = IntToStr(param) + " sek";
    FFT_dane();
    Rys_dane();
}
//---------------------------------------------------------------------------







void __fastcall TForm1::Item_95hzClick(TObject *Sender)
{
        Item_95hz->Checked = true;        
}
//---------------------------------------------------------------------------

void __fastcall TForm1::Item_190hzClick(TObject *Sender)
{
        Item_190hz->Checked = true;
}
//---------------------------------------------------------------------------

void __fastcall TForm1::Item_380hzClick(TObject *Sender)
{
        Item_380hz->Checked = true;        
}
//---------------------------------------------------------------------------

void __fastcall TForm1::Item_760hzClick(TObject *Sender)
{
        Item_760hz->Checked = true;        
}
//---------------------------------------------------------------------------

void __fastcall TForm1::Item_250dpsClick(TObject *Sender)
{
        Item_250dps->Checked = true;
}
//---------------------------------------------------------------------------

void __fastcall TForm1::Item_500dpsClick(TObject *Sender)
{
        Item_500dps->Checked = true;        
}
//---------------------------------------------------------------------------

void __fastcall TForm1::Item_2000dpsClick(TObject *Sender)
{
        Item_2000dps->Checked = true;        
}
//---------------------------------------------------------------------------

void __fastcall TForm1::SendConfClick(TObject *Sender)
{
        if(Item_95hz->Checked)
                RS_data = DATARATE_BIT_95;
        else if(Item_190hz->Checked)
                RS_data = DATARATE_BIT_190;
        else if(Item_380hz->Checked)
                RS_data = DATARATE_BIT_380;
        else if(Item_760hz->Checked)
                RS_data = DATARATE_BIT_760;

        if(Item_250dps->Checked)
                RS_data |= FULLSCALE_BIT_250;
        else if(Item_500dps->Checked)
                RS_data |= FULLSCALE_BIT_500;
        else if(Item_2000dps->Checked)
                RS_data |= FULLSCALE_BIT_2000;

        if(b_odbior)
                RS_data |= 1<<TRANSMITION_BIT;
        else
        {
                Timer1->Enabled = true;
        }
        
        WriteFile(hCommDev, &RS_data, 1,&ile_RS , 0);


}
//-------------------------------------------------------------------------

void __fastcall TForm1::show_config()
{
        switch(data[1] & FULLSCALE_MASK)
        {
            case FULLSCALE_BIT_250:
            {

                gyro_scale = 250;
                FullScaleLabel->Caption = "250 dps";
                break;
            }
            case FULLSCALE_BIT_500:
            {
                gyro_scale = 500;
                FullScaleLabel->Caption = "500 dps";
                break;
            }
            case FULLSCALE_BIT_2000:
            {
                gyro_scale = 2000;
                FullScaleLabel->Caption = "2000 dps";
                break;
            }
        }

        switch(data[1] & DATARATE_MASK)
        {
            case DATARATE_BIT_95:
            {
                gyro_speed = 95;          //92.8 Hz
                DataRateLabel->Caption = "95 Hz";
                zmiana_zakresu();
                break;
            }
            case DATARATE_BIT_190:
            {
                gyro_speed = 190;      //185.7 Hz
                DataRateLabel->Caption = "190 Hz";
                zmiana_zakresu();
                break;
            }
            case DATARATE_BIT_380:
            {
                gyro_speed = 380;         //371.3 Hz
                DataRateLabel->Caption = "380 Hz";
                zmiana_zakresu();
                break;
            }
            case DATARATE_BIT_760:
            {
                gyro_speed = 760;         //742.7 Hz
                DataRateLabel->Caption = "760 Hz";
                zmiana_zakresu();
                break;
            }
        }
}

void __fastcall TForm1::Reset1Click(TObject *Sender)
{
        for(int i=0; i<max_danych; i++)
        {
            data_x[i] = 0;
            data_y[i] = 0;
            data_z[i] = 0;
        }
        Czysc_rys();
}
//---------------------------------------------------------------------------


void __fastcall TForm1::Skanuj1Click(TObject *Sender)
{
        AnsiString znaki;
        HANDLE hCommDev;

        ComPorts->Clear();
        ComPorts->Text = "Porty COM";

        for(int i = 0; i < 256; i++)
        {
            znaki = "COM" + IntToStr(i);
            strcpy(COMchar, znaki.c_str());
            lpFileName = COMchar;
            hCommDev = CreateFile(lpFileName, GENERIC_WRITE |     // OTWARCIE PORTU
                GENERIC_READ, 0, NULL,
                OPEN_EXISTING, 0, NULL);

            if(hCommDev != INVALID_HANDLE_VALUE && hCommDev != NULL)
            {
                ComPorts->Items->Add(znaki);
            }

            CloseHandle(hCommDev);

        }
}
//---------------------------------------------------------------------------

void __fastcall TForm1::zmiana_zakresu()
{
        Zakres11->Caption = "0 - "+FloatToStr((float)gyro_speed/2)+" Hz";
        Zakres21->Caption = "0 - "+FloatToStr((float)gyro_speed/4)+" Hz";
        Zakres31->Caption = "0 - "+FloatToStr((float)gyro_speed/8)+" Hz";
        Zakres41->Caption = "0 - "+FloatToStr((float)gyro_speed/16)+" Hz";
        Zakres51->Caption = "0 - "+FloatToStr((float)gyro_speed/32)+" Hz";
}
//---------------------------------------------------------------------------


void __fastcall TForm1::Zakres11Click(TObject *Sender)
{
        Zakres11->Checked = true;
        dzielnik_fft = 2;
        Rys_dane();
}
//---------------------------------------------------------------------------

void __fastcall TForm1::Zakres21Click(TObject *Sender)
{
        Zakres21->Checked = true;
        dzielnik_fft = 4;;
        Rys_dane();
}
//---------------------------------------------------------------------------

void __fastcall TForm1::Zakres31Click(TObject *Sender)
{
        Zakres31->Checked = true;
        dzielnik_fft = 8;
        Rys_dane();
}
//---------------------------------------------------------------------------

void __fastcall TForm1::Zakres41Click(TObject *Sender)
{
        Zakres41->Checked = true;
        dzielnik_fft = 16;
        Rys_dane();
}
//---------------------------------------------------------------------------

void __fastcall TForm1::Zakres51Click(TObject *Sender)
{
        Zakres51->Checked = true;
        dzielnik_fft = 32;
        Rys_dane();
}
//---------------------------------------------------------------------------

void __fastcall TForm1::Siatka1Click(TObject *Sender)
{
        Rys_dane();
}
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------


void __fastcall TForm1::N1Hz1Click(TObject *Sender)
{
        odstepy = 1;
        Rys_dane();
}
//---------------------------------------------------------------------------

void __fastcall TForm1::N2Hz1Click(TObject *Sender)
{
        odstepy = 2;
        Rys_dane();
}
//---------------------------------------------------------------------------

void __fastcall TForm1::N5Hz1Click(TObject *Sender)
{
        odstepy = 5;
        Rys_dane();
}
//---------------------------------------------------------------------------

void __fastcall TForm1::N10Hz1Click(TObject *Sender)
{
        odstepy = 10;
        Rys_dane();
}
//---------------------------------------------------------------------------

void __fastcall TForm1::N20Hz1Click(TObject *Sender)
{
        odstepy = 20;
        Rys_dane();
}
//---------------------------------------------------------------------------

void __fastcall TForm1::N40Hz1Click(TObject *Sender)
{
        odstepy = 40;
        Rys_dane();
}
//---------------------------------------------------------------------------







