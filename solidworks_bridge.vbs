Option Explicit

Dim args, sourceFile, outputFile
Dim swApp, swModel, swActiveModel, loadErrors, loadWarnings
Dim saveErrors, saveWarnings, result
Dim docType, startedByUs
Dim errorNumber, errorDescription

Set args = WScript.Arguments
If args.Count < 2 Then
    WScript.Echo "ERR|Usage: solidworks_bridge.vbs source_file output_file"
    WScript.Quit 2
End If

sourceFile = args(0)
outputFile = args(1)
startedByUs = False

On Error Resume Next
Err.Clear
Set swApp = GetObject("", "SldWorks.Application")
If Err.Number <> 0 Or swApp Is Nothing Then
    Err.Clear
    Set swApp = CreateObject("SldWorks.Application")
    startedByUs = True
End If

If Err.Number <> 0 Or swApp Is Nothing Then
    errorNumber = Err.Number
    errorDescription = Err.Description
    WScript.Echo "ERR|SOLIDWORKS could not be started. COM=" & errorNumber & "|" & errorDescription
    WScript.Quit 10
End If

Err.Clear
swApp.Visible = True
WScript.Sleep 1500

docType = 1
If LCase(Right(sourceFile, 6)) = "sldasm" Then docType = 2

loadErrors = 0
loadWarnings = 0
Err.Clear
Set swModel = swApp.OpenDoc6(sourceFile, docType, 1, "", loadErrors, loadWarnings)

If swModel Is Nothing Or Err.Number <> 0 Then
    errorNumber = Err.Number
    errorDescription = Err.Description
    WScript.Echo "ERR|SOLIDWORKS could not open the file. LoadErrors=" & loadErrors & "; Warnings=" & loadWarnings & "; COM=" & errorNumber & "|" & errorDescription
    If startedByUs Then swApp.ExitApp
    WScript.Quit 11
End If

Err.Clear
swApp.ActivateDoc3 swModel.GetTitle, True, 0
WScript.Sleep 500

' SOLIDWORKS requires the document being converted to be the active document.
Set swActiveModel = swApp.ActiveDoc

If swActiveModel Is Nothing Or Err.Number <> 0 Then
    errorNumber = Err.Number
    errorDescription = Err.Description
    WScript.Echo "ERR|Could not obtain active SolidWorks document. COM=" & errorNumber & "|" & errorDescription
    swApp.CloseDoc swModel.GetTitle
    If startedByUs Then swApp.ExitApp
    WScript.Quit 12
End If

Err.Clear
swActiveModel.ClearSelection2 True
If docType = 2 Then
    swActiveModel.ForceRebuild3 False
    WScript.Sleep 500
End If

saveErrors = 0
saveWarnings = 0
Err.Clear
result = swActiveModel.Extension.SaveAs2(outputFile, 0, 1, Nothing, "", False, saveErrors, saveWarnings)

If result <> True Or Err.Number <> 0 Then
    errorNumber = Err.Number
    errorDescription = Err.Description
    WScript.Echo "ERR|SOLIDWORKS export to STL failed. SaveErrors=" & saveErrors & "; Warnings=" & saveWarnings & "; COM=" & errorNumber & "|" & errorDescription
    swApp.CloseDoc swModel.GetTitle
    If startedByUs Then swApp.ExitApp
    WScript.Quit 13
End If

swApp.CloseDoc swModel.GetTitle
If startedByUs Then swApp.ExitApp

WScript.Echo "OK|" & loadErrors & "|" & loadWarnings
WScript.Quit 0
