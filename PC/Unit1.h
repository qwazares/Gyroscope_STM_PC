//---------------------------------------------------------------------------

#ifndef Unit1H
#define Unit1H
//---------------------------------------------------------------------------
#include <Classes.hpp>
#include <Controls.hpp>
#include <StdCtrls.hpp>
#include <Forms.hpp>
#include <ExtCtrls.hpp>
#include <Buttons.hpp>
#include "PERFGRAP.h"
#include "FFT_.h"
#include <Menus.hpp>
//---------------------------------------------------------------------------
class TForm1 : public TForm
{
__published:	// IDE-managed Components
        TButton *Open_Close;
        TButton *B_odbierz;
        TLabel *Label1;
        TLabel *Label2;
        TLabel *Label3;
        TEdit *EX;
        TEdit *EY;
        TEdit *EZ;
        TTimer *Timer1;
        TImage *Image1;
        TImage *Image2;
        TImage *Image3;
        TLabel *Label4;
        TLabel *Label5;
        TLabel *Label6;
        TImage *Image4;
        TImage *Image5;
        TImage *Image6;
        TLabel *Author;
        TMainMenu *MainMenu1;
        TMenuItem *Main1;
        TMenuItem *N10241;
        TMenuItem *N20481;
        TMenuItem *N40961;
        TMenuItem *N5121;
        TMenuItem *GyroscopeMenu;
        TMenuItem *DataRateMenu;
        TMenuItem *FullScaleMenu;
        TMenuItem *SendConf;
        TMenuItem *Item_250dps;
        TMenuItem *Item_500dps;
        TMenuItem *Item_2000dps;
        TMenuItem *Item_95hz;
        TMenuItem *Item_190hz;
        TMenuItem *Item_380hz;
        TMenuItem *Item_760hz;
        TLabel *DataRateLabel;
        TLabel *FullScaleLabel;
        TLabel *Label7;
        TMenuItem *Reset1;
        TMenuItem *FFT1;
        TComboBox *ComPorts;
        TMenuItem *Skanuj1;
        TMenuItem *Zakres11;
        TMenuItem *Zakres21;
        TMenuItem *Zakres31;
        TMenuItem *Zakres41;
        TMenuItem *Zakres51;
        TCheckBox *Siatka1;
        TLabel *Label8;
        TLabel *Label9;
        TImage *Image7;
        TImage *Image8;
        TMenuItem *Podziaka1;
        TMenuItem *N1Hz1;
        TMenuItem *N2Hz1;
        TMenuItem *N5Hz1;
        TMenuItem *N10Hz1;
        TMenuItem *N20Hz1;
        TMenuItem *N40Hz1;
        void __fastcall Open_CloseClick(TObject *Sender);
        void __fastcall B_odbierzClick(TObject *Sender);
        void __fastcall Timer1Timer(TObject *Sender);
        void __fastcall FormClose(TObject *Sender, TCloseAction &Action);
        void __fastcall Czysc_rys();
        void __fastcall Rys_dane();
        void __fastcall FFT_dane();
        void __fastcall show_config();
        void __fastcall zmiana_zakresu();
        void __fastcall bitrev(Tablica *dane);
        void __fastcall FFT(Tablica *dane,short IFFT);
        void __fastcall N5121Click(TObject *Sender);
        void __fastcall N10241Click(TObject *Sender);
        void __fastcall N20481Click(TObject *Sender);
        void __fastcall N40961Click(TObject *Sender);
        void __fastcall Item_95hzClick(TObject *Sender);
        void __fastcall Item_190hzClick(TObject *Sender);
        void __fastcall Item_380hzClick(TObject *Sender);
        void __fastcall Item_760hzClick(TObject *Sender);
        void __fastcall Item_250dpsClick(TObject *Sender);
        void __fastcall Item_500dpsClick(TObject *Sender);
        void __fastcall Item_2000dpsClick(TObject *Sender);
        void __fastcall SendConfClick(TObject *Sender);
        void __fastcall Reset1Click(TObject *Sender);
        void __fastcall Skanuj1Click(TObject *Sender);
        void __fastcall Zakres11Click(TObject *Sender);
        void __fastcall Zakres21Click(TObject *Sender);
        void __fastcall Zakres31Click(TObject *Sender);
        void __fastcall Zakres41Click(TObject *Sender);
        void __fastcall Zakres51Click(TObject *Sender);
        void __fastcall Siatka1Click(TObject *Sender);
        void __fastcall N1Hz1Click(TObject *Sender);
        void __fastcall N2Hz1Click(TObject *Sender);
        void __fastcall N5Hz1Click(TObject *Sender);
        void __fastcall N10Hz1Click(TObject *Sender);
        void __fastcall N20Hz1Click(TObject *Sender);
        void __fastcall N40Hz1Click(TObject *Sender);
private:	// User declarations
public:		// User declarations
        __fastcall TForm1(TComponent* Owner);
};
//---------------------------------------------------------------------------
extern PACKAGE TForm1 *Form1;
//---------------------------------------------------------------------------
#endif
