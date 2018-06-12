VERSION 5.00
Begin VB.Form Form1 
   AutoRedraw      =   -1  'True
   BackColor       =   &H00E0E0E0&
   BorderStyle     =   1  'Fixed Single
   Caption         =   "FlexRII序列号与自动编译程序"
   ClientHeight    =   6000
   ClientLeft      =   45
   ClientTop       =   390
   ClientWidth     =   9600
   BeginProperty Font 
      Name            =   "黑体"
      Size            =   15
      Charset         =   134
      Weight          =   400
      Underline       =   0   'False
      Italic          =   0   'False
      Strikethrough   =   0   'False
   EndProperty
   LinkTopic       =   "Form1"
   MaxButton       =   0   'False
   MinButton       =   0   'False
   ScaleHeight     =   400
   ScaleMode       =   3  'Pixel
   ScaleWidth      =   640
   StartUpPosition =   2  '屏幕中心
   Begin VB.Frame Frame3 
      BackColor       =   &H00E0E0E0&
      Caption         =   "编译"
      Height          =   1455
      Left            =   120
      TabIndex        =   2
      Top             =   3240
      Width           =   9375
      Begin VB.CommandButton Command1 
         BackColor       =   &H008080FF&
         Caption         =   "开始编译"
         Height          =   615
         Left            =   3600
         Style           =   1  'Graphical
         TabIndex        =   16
         Top             =   600
         Width           =   1815
      End
   End
   Begin VB.Frame Frame2 
      BackColor       =   &H00E0E0E0&
      Caption         =   "查询序列号合法性(输入15位序列号，只能0-9数字)"
      Height          =   1455
      Left            =   120
      TabIndex        =   1
      Top             =   1680
      Width           =   9375
      Begin VB.CommandButton CommandRequire 
         BackColor       =   &H00E0E0E0&
         Caption         =   "查询"
         Height          =   375
         Left            =   6600
         Style           =   1  'Graphical
         TabIndex        =   4
         Top             =   600
         Width           =   1455
      End
      Begin VB.TextBox TextSerialNumber 
         BackColor       =   &H00000000&
         ForeColor       =   &H00FFFFFF&
         Height          =   420
         Left            =   1200
         TabIndex        =   3
         Text            =   "180620014000500"
         Top             =   600
         Width           =   4575
      End
   End
   Begin VB.Frame Frame1 
      BackColor       =   &H00E0E0E0&
      Caption         =   "生成序列号"
      Height          =   1455
      Left            =   120
      TabIndex        =   0
      Top             =   120
      Width           =   9375
      Begin VB.CommandButton CommandGenerate 
         BackColor       =   &H00E0E0E0&
         Caption         =   "生成"
         Height          =   375
         Left            =   7200
         Style           =   1  'Graphical
         TabIndex        =   15
         Top             =   840
         Width           =   1455
      End
      Begin VB.TextBox TextNum 
         Alignment       =   2  'Center
         BackColor       =   &H00000000&
         ForeColor       =   &H00FFFFFF&
         Height          =   420
         Left            =   5520
         TabIndex        =   9
         Text            =   "50"
         Top             =   840
         Width           =   1215
      End
      Begin VB.TextBox TextBatch 
         Alignment       =   2  'Center
         BackColor       =   &H00000000&
         ForeColor       =   &H00FFFFFF&
         Height          =   420
         Left            =   4200
         TabIndex        =   8
         Text            =   "1"
         Top             =   840
         Width           =   1215
      End
      Begin VB.TextBox TextDay 
         Alignment       =   2  'Center
         BackColor       =   &H00000000&
         ForeColor       =   &H00FFFFFF&
         Height          =   420
         Left            =   2880
         TabIndex        =   7
         Text            =   "20"
         Top             =   840
         Width           =   1215
      End
      Begin VB.TextBox TextMonth 
         Alignment       =   2  'Center
         BackColor       =   &H00000000&
         ForeColor       =   &H00FFFFFF&
         Height          =   420
         Left            =   1560
         TabIndex        =   6
         Text            =   "6"
         Top             =   840
         Width           =   1215
      End
      Begin VB.TextBox TextYear 
         Alignment       =   2  'Center
         BackColor       =   &H00000000&
         ForeColor       =   &H00FFFFFF&
         Height          =   420
         Left            =   240
         TabIndex        =   5
         Text            =   "2018"
         Top             =   840
         Width           =   1215
      End
      Begin VB.Label Label5 
         Alignment       =   2  'Center
         BackStyle       =   0  'Transparent
         Caption         =   "多少个"
         ForeColor       =   &H00000000&
         Height          =   375
         Left            =   5520
         TabIndex        =   14
         Top             =   360
         Width           =   1215
      End
      Begin VB.Label Label4 
         Alignment       =   2  'Center
         BackStyle       =   0  'Transparent
         Caption         =   "第几批"
         ForeColor       =   &H00000000&
         Height          =   375
         Left            =   4200
         TabIndex        =   13
         Top             =   360
         Width           =   1215
      End
      Begin VB.Label Label3 
         Alignment       =   2  'Center
         BackStyle       =   0  'Transparent
         Caption         =   "日"
         ForeColor       =   &H00000000&
         Height          =   375
         Left            =   2880
         TabIndex        =   12
         Top             =   360
         Width           =   1215
      End
      Begin VB.Label Label2 
         Alignment       =   2  'Center
         BackStyle       =   0  'Transparent
         Caption         =   "月"
         ForeColor       =   &H00000000&
         Height          =   375
         Left            =   1560
         TabIndex        =   11
         Top             =   360
         Width           =   1215
      End
      Begin VB.Label Label1 
         Alignment       =   2  'Center
         BackStyle       =   0  'Transparent
         Caption         =   "年"
         ForeColor       =   &H00000000&
         Height          =   375
         Left            =   240
         TabIndex        =   10
         Top             =   360
         Width           =   1215
      End
   End
End
Attribute VB_Name = "Form1"
Attribute VB_GlobalNameSpace = False
Attribute VB_Creatable = False
Attribute VB_PredeclaredId = True
Attribute VB_Exposed = False
Option Explicit
DefLng A-Z
Dim I, J, K, L, R, R0, R1, R2, R3, R4, R5
Dim ValidationMatrix(15, 15) As Byte
Dim PrimeFactor(14) As Integer
Dim OutputSerialNumbers(32767) As String

Private Sub Command1_Click()
 '开始编译
 'Call Shell("cmd /k python validate_serial.py ""180522011000001""", vbNormalFocus)
 Call Shell("cmd /c python build.py", vbNormalFocus)
End Sub

Private Sub CommandGenerate_Click()
 On Error GoTo ErrHandle
 '生成序列号
 Dim SerialNumbersNum As Integer
 Dim SerialNumberString As String
 Dim TempPrimeFactor As Integer
 Dim TempMatrixFactor As Byte
 Dim TempSerialDigit As Byte
 
 Dim OutputNumberString As String
 
 Dim TempYear As Integer
 Dim TempMonth As Byte
 Dim TempDay As Byte
 Dim TempBatch As Byte
 
 SerialNumbersNum = CInt(TextNum.Text)
 If SerialNumbersNum > 32000 Then
  MsgBox "一次要生产的设备数量太多了！", vbCritical, "产能有限"
  Exit Sub
 End If
 Erase OutputSerialNumbers
 For R = 1 To SerialNumbersNum
  
  TempYear = CInt(TextYear.Text)
  TempMonth = CInt(TextMonth.Text)
  TempDay = CInt(TextDay.Text)
  TempBatch = CInt(TextBatch.Text)
  
  TempYear = (TempYear Mod 100)
  If TempMonth < 1 Or TempMonth > 12 Then
   MsgBox "日期、批次输入有误", vbExclamation, "注意"
  End If
  If TempDay < 1 Or TempMonth > 31 Then
   MsgBox "日期、批次输入有误", vbExclamation, "注意"
  End If
  
  SerialNumberString = Format(TempYear, "00") & Format(TempMonth, "00") & Format(TempDay, "00") + Format(TempBatch, "00")
  'Debug.Print SerialNumberString
  
  If Len(SerialNumberString) <> 8 Then
   MsgBox "日期、批次输入有误", vbExclamation, "注意"
   Exit Sub
  End If
  
  SerialNumberString = SerialNumberString & Format(R, "000000")
  'Debug.Print SerialNumberString
  
  TempPrimeFactor = 0
  For R0 = 0 To 13
   TempSerialDigit = CInt(Mid(SerialNumberString, R0 + 1, 1))
   TempPrimeFactor = TempPrimeFactor + TempSerialDigit * PrimeFactor(R0)
  Next R0
  TempPrimeFactor = (TempPrimeFactor Mod 10)
  SerialNumberString = SerialNumberString & Format(TempPrimeFactor, "0")
  'Debug.Print SerialNumberString
  
  OutputNumberString = ""
  For J = 0 To 14
   TempMatrixFactor = 0
   For K = 0 To 14
    TempSerialDigit = CInt(Mid(SerialNumberString, K + 1, 1))
    TempMatrixFactor = TempMatrixFactor + TempSerialDigit * ValidationMatrix(K, J)
   Next K
   OutputNumberString = OutputNumberString & Format(TempMatrixFactor, "0")
  Next J
  OutputSerialNumbers(R) = OutputNumberString
  'Debug.Print OutputNumberString
  'Debug.Print ""
 Next R
 
 Kill "SerialNumbers.txt"
FileGoNext:
 Open "SerialNumbers.txt" For Output As #1
  For R = 1 To SerialNumbersNum
   Print #1, OutputSerialNumbers(R)
  Next R
 Close #1
 'TextSerialFile.Text = "SerialNumbers.txt"
 'Command1.Enabled = True
 
 MsgBox "序列号已生成", vbInformation, "完成"
 
 Exit Sub

ErrHandle:
 'Debug.Print Err.Number
 
 Select Case Err.Number
 Case 6
  MsgBox "一次要生产的设备数量太多了！", vbCritical, "产能有限"
 Case 13
  MsgBox "日期、批次或数量", vbExclamation, "注意"
 Case 53
  GoTo FileGoNext
 End Select

End Sub

Private Sub CommandRequire_Click()
 Dim SerialNumberString As String
 Dim OriginalNumberString As String
 Dim TempPrimeFactor As Integer
 Dim TempMatrixFactor As Byte
 Dim TempSerialDigit As Byte
 
 SerialNumberString = TextSerialNumber.Text
 If Len(SerialNumberString) <> 15 Then
  MsgBox "序列号输入有误", vbExclamation, "注意"
  Exit Sub
 End If
 
 OriginalNumberString = ""
 For J = 0 To 14
  TempMatrixFactor = 0
  For K = 0 To 14
   TempSerialDigit = CInt(Mid(SerialNumberString, K + 1, 1))
   TempMatrixFactor = TempMatrixFactor + TempSerialDigit * ValidationMatrix(J, K)
  Next K
  OriginalNumberString = OriginalNumberString & Format(TempMatrixFactor, "0")
 Next J
 'Debug.Print OriginalNumberString
 
 TempPrimeFactor = 0
 For R0 = 0 To 13
  TempSerialDigit = CInt(Mid(OriginalNumberString, R0 + 1, 1))
  TempPrimeFactor = TempPrimeFactor + TempSerialDigit * PrimeFactor(R0)
 Next R0
 TempPrimeFactor = (TempPrimeFactor Mod 10)
 TempSerialDigit = CInt(Mid(OriginalNumberString, 15, 1))
 If TempSerialDigit <> TempPrimeFactor Then
  MsgBox "序列号非法！", vbCritical, "非法"
 Else
  MsgBox "序列号合法。", vbInformation, "合法"
 End If
 
End Sub

Private Sub Form_Load()
 '整理验证矩阵
 'validationMap.append ([1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
 'validationMap.append ([0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
 'validationMap.append ([0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
 'validationMap.append ([0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
 'validationMap.append ([0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
 'validationMap.append ([0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0])
 'validationMap.append ([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0])
 'validationMap.append ([0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0])
 'validationMap.append ([0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0])
 'validationMap.append ([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0])
 'validationMap.append ([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0])
 'validationMap.append ([0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0])
 'validationMap.append ([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0])
 'validationMap.append ([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1])
 'validationMap.append ([0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0])
 ValidationMatrix(0, 0) = 1
 ValidationMatrix(1, 1) = 1
 ValidationMatrix(2, 2) = 1
 ValidationMatrix(3, 3) = 1
 ValidationMatrix(4, 4) = 1
 ValidationMatrix(5, 5) = 1
 ValidationMatrix(6, 10) = 1
 ValidationMatrix(7, 7) = 1
 ValidationMatrix(8, 6) = 1
 ValidationMatrix(9, 13) = 1
 ValidationMatrix(10, 11) = 1
 ValidationMatrix(11, 9) = 1
 ValidationMatrix(12, 12) = 1
 ValidationMatrix(13, 14) = 1
 ValidationMatrix(14, 8) = 1
 
 '整理验证码计算因子
 PrimeFactor(0) = 2
 PrimeFactor(1) = 3
 PrimeFactor(2) = 5
 PrimeFactor(3) = 7
 PrimeFactor(4) = 11
 PrimeFactor(5) = 13
 PrimeFactor(6) = 17
 PrimeFactor(7) = 19
 PrimeFactor(8) = 23
 PrimeFactor(9) = 29
 PrimeFactor(10) = 31
 PrimeFactor(11) = 37
 PrimeFactor(12) = 41
 PrimeFactor(13) = 43
End Sub

Private Sub Form_Unload(Cancel As Integer)
 End
End Sub
