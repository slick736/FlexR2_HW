VERSION 5.00
Begin VB.Form Form1 
   AutoRedraw      =   -1  'True
   BackColor       =   &H00E0E0E0&
   BorderStyle     =   1  'Fixed Single
   Caption         =   "FlexRII���к����Զ��������"
   ClientHeight    =   6000
   ClientLeft      =   45
   ClientTop       =   390
   ClientWidth     =   9600
   BeginProperty Font 
      Name            =   "����"
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
   StartUpPosition =   2  '��Ļ����
   Begin VB.Frame Frame4 
      BackColor       =   &H00E0E0E0&
      Height          =   5775
      Left            =   9000
      TabIndex        =   17
      Top             =   5640
      Width           =   9375
      Begin VB.PictureBox Picture1 
         BackColor       =   &H00000000&
         Height          =   495
         Left            =   840
         ScaleHeight     =   29
         ScaleMode       =   3  'Pixel
         ScaleWidth      =   509
         TabIndex        =   19
         Top             =   3120
         Width           =   7695
         Begin VB.Shape Shape1 
            BorderColor     =   &H00FFFFFF&
            FillColor       =   &H00FFFFFF&
            FillStyle       =   0  'Solid
            Height          =   255
            Left            =   120
            Top             =   120
            Width           =   255
         End
      End
      Begin VB.Label Label7 
         BackStyle       =   0  'Transparent
         Caption         =   "���ǰ�벻Ҫ�Լ���������κ���������"
         Height          =   375
         Left            =   1680
         TabIndex        =   21
         Top             =   2040
         Width           =   5655
      End
      Begin VB.Label Label6 
         BackStyle       =   0  'Transparent
         Caption         =   "�����������к�EXCEL�ĵ������Ժ�"
         Height          =   375
         Left            =   2040
         TabIndex        =   18
         Top             =   1440
         Width           =   5175
      End
   End
   Begin VB.Frame Frame3 
      BackColor       =   &H00E0E0E0&
      Caption         =   "����"
      Height          =   1215
      Left            =   120
      TabIndex        =   2
      Top             =   4320
      Width           =   9375
      Begin VB.CommandButton Command2 
         BackColor       =   &H0080FFFF&
         Caption         =   "�����ϴ�δ��ɵı���"
         Enabled         =   0   'False
         Height          =   615
         Left            =   4440
         Style           =   1  'Graphical
         TabIndex        =   20
         Top             =   360
         Width           =   3615
      End
      Begin VB.CommandButton Command1 
         BackColor       =   &H008080FF&
         Caption         =   "��ʼ����"
         Height          =   615
         Left            =   1320
         Style           =   1  'Graphical
         TabIndex        =   16
         Top             =   360
         Width           =   1815
      End
   End
   Begin VB.Frame Frame2 
      BackColor       =   &H00E0E0E0&
      Caption         =   "��ѯ���кźϷ���(����15λ���кţ�ֻ��0-9����)"
      Height          =   1215
      Left            =   120
      TabIndex        =   1
      Top             =   2880
      Width           =   9375
      Begin VB.CommandButton CommandRequire 
         BackColor       =   &H00E0E0E0&
         Caption         =   "��ѯ"
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
      Caption         =   "�������к�"
      Height          =   2535
      Left            =   120
      TabIndex        =   0
      Top             =   120
      Width           =   9375
      Begin VB.TextBox TextSoftWare 
         Alignment       =   2  'Center
         BackColor       =   &H00000000&
         ForeColor       =   &H00FFFFFF&
         Height          =   420
         Left            =   2880
         TabIndex        =   25
         Text            =   "1.0.0.3"
         Top             =   1920
         Width           =   2535
      End
      Begin VB.TextBox TextHardWare 
         Alignment       =   2  'Center
         BackColor       =   &H00000000&
         ForeColor       =   &H00FFFFFF&
         Height          =   420
         Left            =   2880
         TabIndex        =   24
         Text            =   "1.1.0.1"
         Top             =   1440
         Width           =   2535
      End
      Begin VB.CommandButton CommandGenerate 
         BackColor       =   &H00E0E0E0&
         Caption         =   "����"
         Height          =   615
         Left            =   7320
         Style           =   1  'Graphical
         TabIndex        =   15
         Top             =   1680
         Width           =   1695
      End
      Begin VB.TextBox TextNum 
         Alignment       =   2  'Center
         BackColor       =   &H00000000&
         ForeColor       =   &H00FFFFFF&
         Height          =   420
         Left            =   5520
         TabIndex        =   9
         Text            =   "50"
         Top             =   720
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
         Top             =   720
         Width           =   1215
      End
      Begin VB.TextBox TextDay 
         Alignment       =   2  'Center
         BackColor       =   &H00000000&
         ForeColor       =   &H00FFFFFF&
         Height          =   420
         Left            =   2880
         TabIndex        =   7
         Text            =   "30"
         Top             =   720
         Width           =   1215
      End
      Begin VB.TextBox TextMonth 
         Alignment       =   2  'Center
         BackColor       =   &H00000000&
         ForeColor       =   &H00FFFFFF&
         Height          =   420
         Left            =   1560
         TabIndex        =   6
         Text            =   "11"
         Top             =   720
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
         Top             =   720
         Width           =   1215
      End
      Begin VB.Label Label9 
         Alignment       =   2  'Center
         BackStyle       =   0  'Transparent
         Caption         =   "Ƕ��ʽ����汾��"
         ForeColor       =   &H00000000&
         Height          =   375
         Left            =   240
         TabIndex        =   23
         Top             =   1920
         Width           =   2535
      End
      Begin VB.Label Label8 
         Alignment       =   2  'Center
         BackStyle       =   0  'Transparent
         Caption         =   "Ӳ���汾��"
         ForeColor       =   &H00000000&
         Height          =   375
         Left            =   240
         TabIndex        =   22
         Top             =   1440
         Width           =   2535
      End
      Begin VB.Label Label5 
         Alignment       =   2  'Center
         BackStyle       =   0  'Transparent
         Caption         =   "���ٸ�"
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
         Caption         =   "�ڼ���"
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
         Caption         =   "��"
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
         Caption         =   "��"
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
         Caption         =   "��"
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
Dim xlApp As Excel.Application '����EXCEL��
Dim xlBook As Excel.Workbook '���幤������
Dim xlSheet As Excel.Worksheet '���幤������
Const MaxBatchNum = 32000

Private Sub Command1_Click()
 '��ʼ����
 'Call Shell("cmd /k python validate_serial.py ""180522011000001""", vbNormalFocus)
 
 '����ϴα��뵽ʲôλ��
 Dim LineSequence As Integer
 Dim LineSequenceString As String
 Dim LineOverall As Integer
 Dim LineOverallString As String
 
 Dim TYear As String
 Dim TDate As String
 Dim TBatch As String
 
 Dim MsgBoxResult
 Dim MsgBoxPrompt As String
 
 Open "Sequence.txt" For Input As #1
  Input #1, LineSequenceString
  Input #1, LineOverallString
  Input #1, TYear
  Input #1, TDate
  Input #1, TBatch
 Close #1
 LineSequence = Int(Val(LineSequenceString))
 LineOverall = Int(Val(LineOverallString))
 
 If LineSequence > 0 Then
  '�ϴδ���û�����bin���Ƿ������
  MsgBoxPrompt = "�ϴα���û����ɾͱ�ȡ����" & Chr$(13) & "Ŀǰ���뵽�˵� " & Str(LineSequence) & " ������ " & Str(LineOverall) & " ����" & Chr$(13) & "�Ƿ���Ҫ���¿�ʼ��"
  MsgBoxResult = MsgBox(MsgBoxPrompt, vbQuestion + vbYesNo, "���棺�ϴ�δ���")
  If MsgBoxResult = vbYes Then
   Kill "Sequence.txt"
   Open "Sequence.txt" For Output As #1
    Print #1, "0"
    Print #1, Str(LineOverall)
    Print #1, TYear
    Print #1, TDate
    Print #1, TBatch
   Close #1
  Else
   Exit Sub
  End If
 End If
 
 Command2_Click
End Sub

Private Sub Command2_Click()
 '����
 Dim TempYear As Integer
 Dim TempMonth As Byte
 Dim TempDay As Byte
 Dim TempBatch As Byte
 
 TempYear = CInt(TextYear.Text)
 TempMonth = CInt(TextMonth.Text)
 TempDay = CInt(TextDay.Text)
 TempBatch = CInt(TextBatch.Text)
 
 If TempMonth < 1 Or TempMonth > 12 Then
  MsgBox "���ڡ�������������", vbExclamation, "ע��"
  Exit Sub
 End If
 If TempDay < 1 Or TempMonth > 31 Then
  MsgBox "���ڡ�������������", vbExclamation, "ע��"
  Exit Sub
 End If
 
 '����ѱ�����ļ����Ƿ��Ѵ���
 Dim PathKey As String
 PathKey = "simple_peripheral"
 Dim PathKeyPos As Byte
 PathKeyPos = InStr(App.Path, PathKey) + Len(PathKey)
 Dim TargetFolder As String
 TargetFolder = Left$(App.Path, PathKeyPos) + "tirtos\iar\app\FlashROM_StackLibrary\Exe\"
 TargetFolder = TargetFolder & Format(TempYear, "0000") & "_" & Format(TempMonth, "00") & Format(TempDay, "00") & "_" & Format(TempBatch, "00")
 Dim MsgBoxResult
 If Dir(TargetFolder, vbDirectory) <> "" Then
  '���ڣ��������±���
  If Command2.Enabled = False Then
   MsgBoxResult = MsgBox("�����ڸ����ε�bin�ļ����Ѵ��ڣ��Ƿ񸲸Ǳ��룿", vbExclamation + vbYesNo, "ע�⣺�ļ����Ѵ���")
   If MsgBoxResult = vbNo Then
    'ȡ�����룺�˳�sub
    Exit Sub
   End If
   If MsgBoxResult = vbYes Then
    '���Ǳ��룺ɾ������ļ��м��Ѿ�����õ�bin
    Kill TargetFolder & "\*.*"
    RmDir TargetFolder
   End If
  End If
 End If
 'Debug.Print TargetFolder
 'Exit Sub
 
 Dim Paras As String
 Paras = """" & Format(TempYear, "0000") & """ """ & Format(TempMonth, "00") & Format(TempDay, "00") & """ """ & Format(TempBatch, "00") & """"
 'Command2.Enabled = True
 
 Call Shell(("cmd /c python build.py " & Paras), vbNormalFocus)
End Sub

Private Sub CommandGenerate_Click()
 On Local Error GoTo ErrHandle
 '�������к�
 Dim SerialNumbersNum As Integer
 Dim SerialNumberString As String
 Dim TempPrimeFactor As Integer
 Dim TempMatrixFactor As Byte
 Dim TempSerialDigit As Byte
 
 Dim OutputNumberString As String
 
 Dim TempYear As Integer
 Dim TempYearOrig As Integer
 Dim TempMonth As Byte
 Dim TempDay As Byte
 Dim TempBatch As Byte
 
 Dim ShapeLeft As Currency
 
 Dim ContinueBuildFlag
 
 '�����кŵ������Ų�Ҫ��
 Dim PreSerialNumber As String
 PreSerialNumber = "(01)06970413500005(11)"
 Dim SerialNumberFirst6 As String
 
 Dim MidSerialNumber As String
 MidSerialNumber = "(21)"
 Dim SerialNumberLast9 As String
 
 Dim CompleteSerialNumber As String
 Dim LastGenerateNum As Integer
 
 '��ʼ������
 SerialNumbersNum = CInt(TextNum.Text)
 If SerialNumbersNum > MaxBatchNum Then
  MsgBox "һ��Ҫ�������豸����̫���ˣ�", vbCritical, "��������"
  Exit Sub
 End If
 Erase OutputSerialNumbers
 For R = 1 To SerialNumbersNum
  
  TempYear = CInt(TextYear.Text)
  TempMonth = CInt(TextMonth.Text)
  TempDay = CInt(TextDay.Text)
  TempBatch = CInt(TextBatch.Text)
  TempYearOrig = TempYear
  
  TempYear = (TempYear Mod 100)
  If TempMonth < 1 Or TempMonth > 12 Then
   MsgBox "���ڡ�������������", vbExclamation, "ע��"
   Exit Sub
  End If
  If TempDay < 1 Or TempMonth > 31 Then
   MsgBox "���ڡ�������������", vbExclamation, "ע��"
   Exit Sub
  End If
  
  SerialNumberString = Format(TempYear, "00") & Format(TempMonth, "00") & Format(TempDay, "00") + Format(TempBatch, "00")
  'Debug.Print SerialNumberString
  
  If Len(SerialNumberString) <> 8 Then
   MsgBox "���ڡ�������������", vbExclamation, "ע��"
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
 
 '��������ϴε�δ������ɼ�¼���;����û�
 If Command2.Enabled = True Then
  ContinueBuildFlag = MsgBox("��һ���ı���δ��ɣ�������һ�������кž�Ҫ���±��룬�Ƿ������", vbExclamation + vbYesNo, "����")
  If ContinueBuildFlag = vbYes Then
   'ǿ���������ɣ�Ĩ���ϴεļ�¼����
   Command2.Enabled = False
  Else
   '����ԭ��¼
   Exit Sub
  End If
 End If
 
 Kill "Sequence.txt"
 Open "Sequence.txt" For Output As #1
  Print #1, "0"
  Print #1, Str(SerialNumbersNum)
  Print #1, Format(TempYearOrig, "0000")
  Print #1, Format(TempMonth, "00") & Format(TempDay, "00")
  Print #1, Format(TempBatch, "00")
 Close #1
 
 Kill "SerialNumbers.txt"
FileGoNext:
 Open "SerialNumbers.txt" For Output As #1
  For R = 1 To SerialNumbersNum
   Print #1, OutputSerialNumbers(R)
  Next R
 Close #1
 'TextSerialFile.Text = "SerialNumbers.txt"
 'Command1.Enabled = True
 
 '���������кţ�ת��Excel
 '����excel����
 Frame4.Visible = True
 Picture1.Visible = False
 Me.Enabled = False
 Me.Visible = True
 Set xlApp = CreateObject("Excel.Application") '����EXCELӦ����
 xlApp.Visible = False
 Set xlBook = xlApp.Workbooks.Open(App.Path & "\serials.xlsx") '��EXCEL������
 Set xlSheet = xlBook.Worksheets(1) '��EXCEL������
 
 '�ȼ���ϴ������˶��ٸ�
 LastGenerateNum = 0
 Do
  If xlSheet.Cells(LastGenerateNum + 2, 1) = "" Then
   Exit Do
  Else
   LastGenerateNum = LastGenerateNum + 1
  End If
 Loop
 
 '����Excel�ڶ���
 Frame4.Visible = True
 Shape1.Left = 8
 ShapeLeft = 0
 Picture1.Visible = True
 Me.Visible = True
 If LastGenerateNum > 0 Then
  For R = 1 To LastGenerateNum
   xlSheet.Cells(R + 1, 1) = ""
   xlSheet.Cells(R + 1, 2) = ""
   xlSheet.Cells(R + 1, 3) = ""
   ShapeLeft = R * 480 / LastGenerateNum
   Shape1.Left = 8 + ShapeLeft
   DoEvents
  Next R
 End If
 
 '��Excel��д���Ӧ����
 Shape1.Left = 8
 ShapeLeft = 0
 For R = 1 To SerialNumbersNum
  '���кŷ�Ϊǰ6��9��������
  SerialNumberFirst6 = Left$(OutputSerialNumbers(R), 6)
  SerialNumberLast9 = Right$(OutputSerialNumbers(R), 9)
  CompleteSerialNumber = PreSerialNumber & SerialNumberFirst6 & MidSerialNumber & SerialNumberLast9
  xlSheet.Cells(R + 1, 1) = R
  xlSheet.Cells(R + 1, 2) = OutputSerialNumbers(R)
  xlSheet.Cells(R + 1, 3) = CompleteSerialNumber
  ShapeLeft = R * 480 / SerialNumbersNum
  Shape1.Left = 8 + ShapeLeft
  DoEvents
 Next R
 
 xlBook.Close (True) '�ر�EXCEL������
 xlApp.Quit '�ر�EXCEL
 Set xlApp = Nothing '�ͷ�EXCEL����
 
 '���ɰ汾��
 Generate_Version
 
 Me.Enabled = True
 Frame4.Visible = False
 MsgBox "���к�������", vbInformation, "���"
 
 Exit Sub

ErrHandle:
 'Debug.Print Err.Number
 
 Select Case Err.Number
 Case 6
  MsgBox "һ��Ҫ�������豸����̫���ˣ�", vbCritical, "��������"
 Case 13
  MsgBox "���ڡ����λ�����", vbExclamation, "ע��"
 Case 53
  GoTo FileGoNext
 End Select

End Sub
Sub Generate_Version()
On Local Error Resume Next
 Kill "Version.txt"
 Open "Version.txt" For Output As #1
  Print #1, TextHardWare.Text
  Print #1, TextSoftWare.Text
 Close #1
End Sub

Private Sub CommandRequire_Click()
 Dim SerialNumberString As String
 Dim OriginalNumberString As String
 Dim TempPrimeFactor As Integer
 Dim TempMatrixFactor As Byte
 Dim TempSerialDigit As Byte
 
 SerialNumberString = TextSerialNumber.Text
 If Len(SerialNumberString) <> 15 Then
  MsgBox "���к���������", vbExclamation, "ע��"
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
  MsgBox "���кŷǷ���", vbCritical, "�Ƿ�"
 Else
  MsgBox "���кźϷ���", vbInformation, "�Ϸ�"
 End If
 
End Sub

Private Sub Form_Load()
 '������֤����
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
 
 '������֤���������
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
 
 'EXCEL������ʾ��
 Frame4.Left = 8
 Frame4.Top = 8
 Frame4.Visible = False
 Shape1.Left = 8
 
 '����ϴ��Ƿ����δ��ɵı���
 Dim LineSequence As Integer
 Dim LineSequenceString As String
 Dim LineOverall As Integer
 Dim LineOverallString As String
 
 'Dim MsgBoxResult
 'Dim MsgBoxPrompt As String
 Dim TYear As String
 Dim TDate As String
 Dim TBatch As String
 Dim TempYear As Integer
 Dim TempMonth As Byte
 Dim TempDay As Byte
 Dim TempBatch As Byte
 
 Open "Sequence.txt" For Input As #1
  Input #1, LineSequenceString
  Input #1, LineOverallString
  Input #1, TYear
  Input #1, TDate
  Input #1, TBatch
 Close #1
 LineSequence = Int(Val(LineSequenceString))
 LineOverall = Int(Val(LineOverallString))
 TempYear = Int(Val(TYear))
 TempMonth = Int(Val(Left$(TDate, 2)))
 TempDay = Int(Val(Right$(TDate, 2)))
 TempBatch = Int(Val(TBatch))
 TextYear.Text = TempYear
 TextMonth.Text = TempMonth
 TextDay.Text = TempDay
 TextBatch.Text = TempBatch
 TextNum.Text = LineOverall
 
 If LineSequence > 0 Then
  Command2.Enabled = True
 Else
  Command2.Enabled = False
  '�ϴδ���û�����bin���Ƿ������
  'MsgBoxPrompt = "�ϴα���û����ɾͱ�ȡ���ˣ�Ŀǰ���뵽�˵� "
  'MsgBoxResult = MsgBox("�ϴα���û����ɾͱ�ȡ���ˣ�Ŀǰ���뵽�˵� ��(�� ��)���Ƿ�������룿(ѡ�������¿�ʼ)", vbQuestion + vbYesNo, "�Ƿ����")
  'If MsgBoxResult = vbNo Then
   'Open "SerialNumbers.txt" For Output As #1
    'Print #1, "0"
    'Print #1, Str(LineOverall)
   'Close #1
  'End If
 End If
End Sub

Private Sub Form_Unload(Cancel As Integer)
 End
End Sub
