//---------------------------------------------------------------------------

#include <vcl.h>
#pragma hdrstop

#include "Unit2.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)
#pragma resource "*.dfm"
TForm2 *Form2;

extern int odstepy;
extern int param;
extern int mala_skala;
//---------------------------------------------------------------------------
__fastcall TForm2::TForm2(TComponent* Owner)
        : TForm(Owner)
{
}
//---------------------------------------------------------------------------

void __fastcall TForm2::FormCreate(TObject *Sender)
{
        Edit1->Text = IntToStr(param) + " sek";
        Edit2->Text = IntToStr(odstepy) + " Hz";
}
//---------------------------------------------------------------------------

void __fastcall TForm2::Button1Click(TObject *Sender)
{

        if(Edit1->Text.ToIntDef(param) != param)
        {
            param =  Edit1->Text.ToIntDef(param);
            mala_skala = 1;
        }
        if(Edit2->Text.ToIntDef(odstepy) != 0)
            odstepy =  Edit2->Text.ToIntDef(odstepy);
}
//---------------------------------------------------------------------------
void __fastcall TForm2::Button2Click(TObject *Sender)
{
        if(Edit1->Text.ToIntDef(odstepy) != 0)
        {
            param =  Edit1->Text.ToIntDef(odstepy);
            mala_skala = 1;
        }
        if(Edit2->Text.ToIntDef(odstepy) != 0)
            odstepy =  Edit2->Text.ToIntDef(odstepy);

        Form2->Close();
}
//---------------------------------------------------------------------------
