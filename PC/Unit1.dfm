object Form1: TForm1
  Left = -4
  Top = 108
  Align = alTop
  BorderIcons = [biSystemMenu, biMinimize]
  BorderStyle = bsSingle
  Caption = #379'yroskop'
  ClientHeight = 444
  ClientWidth = 1348
  Color = clBtnFace
  Font.Charset = DEFAULT_CHARSET
  Font.Color = clWindowText
  Font.Height = -11
  Font.Name = 'MS Sans Serif'
  Font.Style = []
  Menu = MainMenu1
  OldCreateOrder = False
  Position = poDefault
  Visible = True
  WindowState = wsMaximized
  OnClose = FormClose
  PixelsPerInch = 96
  TextHeight = 13
  object Label1: TLabel
    Left = 200
    Top = 32
    Width = 37
    Height = 20
    Caption = 'O'#347' X'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -16
    Font.Name = 'MS Sans Serif'
    Font.Style = []
    ParentFont = False
  end
  object Label2: TLabel
    Left = 384
    Top = 32
    Width = 36
    Height = 20
    Caption = 'O'#347' Y'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -16
    Font.Name = 'MS Sans Serif'
    Font.Style = []
    ParentFont = False
  end
  object Label3: TLabel
    Left = 568
    Top = 32
    Width = 35
    Height = 20
    Caption = 'O'#347' Z'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -16
    Font.Name = 'MS Sans Serif'
    Font.Style = []
    ParentFont = False
  end
  object Image1: TImage
    Left = 40
    Top = 78
    Width = 649
    Height = 209
  end
  object Image2: TImage
    Left = 40
    Top = 287
    Width = 649
    Height = 209
  end
  object Image3: TImage
    Left = 40
    Top = 496
    Width = 649
    Height = 209
  end
  object Label4: TLabel
    Left = 0
    Top = 176
    Width = 37
    Height = 20
    Caption = 'O'#347' X'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -16
    Font.Name = 'MS Sans Serif'
    Font.Style = []
    ParentFont = False
  end
  object Label5: TLabel
    Left = 0
    Top = 400
    Width = 36
    Height = 20
    Caption = 'O'#347' Y'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -16
    Font.Name = 'MS Sans Serif'
    Font.Style = []
    ParentFont = False
  end
  object Label6: TLabel
    Left = 0
    Top = 600
    Width = 35
    Height = 20
    Caption = 'O'#347' Z'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -16
    Font.Name = 'MS Sans Serif'
    Font.Style = []
    ParentFont = False
  end
  object Image4: TImage
    Left = 704
    Top = 78
    Width = 649
    Height = 209
  end
  object Image5: TImage
    Left = 704
    Top = 287
    Width = 649
    Height = 209
  end
  object Image6: TImage
    Left = 704
    Top = 496
    Width = 649
    Height = 209
  end
  object Author: TLabel
    Left = 1208
    Top = 24
    Width = 131
    Height = 16
    Caption = 'Autor: Krzysztof Kope'#263
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -15
    Font.Name = 'MS Sans Serif'
    Font.Style = []
    ParentFont = False
  end
  object DataRateLabel: TLabel
    Left = 950
    Top = 32
    Width = 5
    Height = 24
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -19
    Font.Name = 'MS Sans Serif'
    Font.Style = []
    ParentFont = False
  end
  object FullScaleLabel: TLabel
    Left = 1024
    Top = 32
    Width = 5
    Height = 24
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -19
    Font.Name = 'MS Sans Serif'
    Font.Style = []
    ParentFont = False
  end
  object Label7: TLabel
    Left = 974
    Top = 0
    Width = 78
    Height = 24
    Caption = #379'yroskop'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -21
    Font.Name = 'MS Sans Serif'
    Font.Style = []
    ParentFont = False
  end
  object Label8: TLabel
    Left = 561
    Top = 62
    Width = 32
    Height = 16
    Caption = '1 sek'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -13
    Font.Name = 'MS Sans Serif'
    Font.Style = []
    ParentFont = False
  end
  object Label9: TLabel
    Left = 761
    Top = 62
    Width = 32
    Height = 16
    Caption = '1 sek'
    Font.Charset = DEFAULT_CHARSET
    Font.Color = clWindowText
    Font.Height = -13
    Font.Name = 'MS Sans Serif'
    Font.Style = []
    ParentFont = False
  end
  object Image7: TImage
    Left = 0
    Top = 276
    Width = 1353
    Height = 25
  end
  object Image8: TImage
    Left = 0
    Top = 500
    Width = 1353
    Height = 25
  end
  object Open_Close: TButton
    Left = 8
    Top = 32
    Width = 75
    Height = 25
    Caption = 'Otw'#243'rz port'
    TabOrder = 0
    OnClick = Open_CloseClick
  end
  object B_odbierz: TButton
    Left = 104
    Top = 32
    Width = 75
    Height = 25
    Caption = 'Odbierz'
    Enabled = False
    TabOrder = 1
    OnClick = B_odbierzClick
  end
  object EX: TEdit
    Left = 248
    Top = 32
    Width = 121
    Height = 21
    TabOrder = 2
  end
  object EY: TEdit
    Left = 432
    Top = 32
    Width = 121
    Height = 21
    TabOrder = 3
  end
  object EZ: TEdit
    Left = 616
    Top = 32
    Width = 121
    Height = 21
    TabOrder = 4
  end
  object ComPorts: TComboBox
    Left = 8
    Top = 8
    Width = 145
    Height = 21
    ItemHeight = 13
    TabOrder = 5
    Text = 'Porty COM'
  end
  object Siatka1: TCheckBox
    Left = 752
    Top = 32
    Width = 97
    Height = 17
    Caption = 'Siatka'
    Checked = True
    State = cbChecked
    TabOrder = 6
    OnClick = Siatka1Click
  end
  object Timer1: TTimer
    Enabled = False
    Interval = 20
    OnTimer = Timer1Timer
    Left = 126
    Top = 112
  end
  object MainMenu1: TMainMenu
    Left = 80
    Top = 112
    object Main1: TMenuItem
      Caption = 'Ilo'#347#263' danych'
      object N5121: TMenuItem
        Caption = '512'
        RadioItem = True
        OnClick = N5121Click
      end
      object N10241: TMenuItem
        Caption = '1024'
        Checked = True
        RadioItem = True
        OnClick = N10241Click
      end
      object N20481: TMenuItem
        Caption = '2048'
        RadioItem = True
        OnClick = N20481Click
      end
      object N40961: TMenuItem
        Caption = '4096'
        RadioItem = True
        OnClick = N40961Click
      end
      object Reset1: TMenuItem
        Caption = 'Reset'
        OnClick = Reset1Click
      end
    end
    object GyroscopeMenu: TMenuItem
      Caption = #379'yroskop'
      object DataRateMenu: TMenuItem
        Caption = '&Cz'#281'stotliwo'#347#263' odczytu danych'
        object Item_95hz: TMenuItem
          Caption = '95 Hz'
          RadioItem = True
          OnClick = Item_95hzClick
        end
        object Item_190hz: TMenuItem
          Caption = '190 Hz'
          RadioItem = True
          OnClick = Item_190hzClick
        end
        object Item_380hz: TMenuItem
          Caption = '380 Hz'
          RadioItem = True
          OnClick = Item_380hzClick
        end
        object Item_760hz: TMenuItem
          Caption = '760 Hz'
          Checked = True
          RadioItem = True
          OnClick = Item_760hzClick
        end
      end
      object FullScaleMenu: TMenuItem
        Caption = 'Zakres'
        object Item_250dps: TMenuItem
          Caption = '250 dps'
          RadioItem = True
          OnClick = Item_250dpsClick
        end
        object Item_500dps: TMenuItem
          Caption = '500 dps'
          RadioItem = True
          OnClick = Item_500dpsClick
        end
        object Item_2000dps: TMenuItem
          Caption = '2000 dps'
          Checked = True
          RadioItem = True
          OnClick = Item_2000dpsClick
        end
      end
      object SendConf: TMenuItem
        Caption = 'Ustaw'
        Enabled = False
        OnClick = SendConfClick
      end
    end
    object FFT1: TMenuItem
      Caption = 'FFT'
      object Zakres11: TMenuItem
        Caption = 'Zakres1'
        Checked = True
        RadioItem = True
        OnClick = Zakres11Click
      end
      object Zakres21: TMenuItem
        Caption = 'Zakres2'
        RadioItem = True
        OnClick = Zakres21Click
      end
      object Zakres31: TMenuItem
        Caption = 'Zakres3'
        RadioItem = True
        OnClick = Zakres31Click
      end
      object Zakres41: TMenuItem
        Caption = 'Zakres4'
        RadioItem = True
        OnClick = Zakres41Click
      end
      object Zakres51: TMenuItem
        Caption = 'Zakres5'
        RadioItem = True
        OnClick = Zakres51Click
      end
    end
    object Podziaka1: TMenuItem
      Caption = 'Podzia'#322'ka'
      object N1Hz1: TMenuItem
        Caption = '1 Hz'
        OnClick = N1Hz1Click
      end
      object N2Hz1: TMenuItem
        Caption = '2 Hz'
        OnClick = N2Hz1Click
      end
      object N5Hz1: TMenuItem
        Caption = '5 Hz'
        OnClick = N5Hz1Click
      end
      object N10Hz1: TMenuItem
        Caption = '10 Hz'
        OnClick = N10Hz1Click
      end
      object N20Hz1: TMenuItem
        Caption = '20 Hz'
        OnClick = N20Hz1Click
      end
      object N40Hz1: TMenuItem
        Caption = '40 Hz'
        OnClick = N40Hz1Click
      end
    end
    object Skanuj1: TMenuItem
      Caption = 'Skanuj'
      OnClick = Skanuj1Click
    end
  end
end
