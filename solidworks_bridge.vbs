Option Explicit

Dim args, sourceFile, outputFile
Dim swApp, swModel, loadErrors, loadWarnings
Dim saveErrors, saveWarnings, result
Dim docType, startedByUs

Set args = WScript.Arguments
If args.Count < 2 Then
    WScript.Echo "ERR|Usage: solidworks_bridge.vbs source_file output_file"
    WScript.Quit 2
End If

sourceFile = args(0)
outputFile = args(1)
startedByUs = False

On Error Resume Next
Set swApp = GetObject("", "SldWorks.Application")
If Err.Number <> 0 Or swApp Is Nothing Then
    Err.Clear
    Set swApp = CreateObject("SldWorks.Application")
    startedByUs = True
End If

If Err.Number <> 0 Or swApp Is Nothing Then
    WScript.Echo "ERR|SOLIDWORKS could not be started. Install SOLIDWORKS on this Windows PC."
    WScript.Quit 10
End If

If startedByUs Then swApp.Visible = False

docType = 1
If LCase(Right(sourceFile, 6)) = "sldasm" Then docType = 2

loadErrors = 0
loadWarnings = 0
Set swModel = swApp.OpenDoc6(sourceFile, docType, 1, "", loadErrors, loadWarnings)

If swModel Is Nothing Then
    WScript.Echo "ERR|SOLIDWORKS could not open the file. LoadErrors=" & loadErrors & "; Warnings=" & loadWarnings
    If startedByUs Then swApp.ExitApp
    WScript.Quit 11
End If

swApp.ActivateDoc3 swModel.GetTitle, True, 0
swModel.ClearSelection2 True

If docType = 2 Then
    Err.Clear
    swModel.ResolveAllLightWeightComponents False
    Err.Clear
End If

saveErrors = 0
saveWarnings = 0
Err.Clear
result = swModel.Extension.SaveAs2(outputFile, 0, 1, Nothing, "", False, saveErrors, saveWarnings)

If result <> True Or Err.Number <> 0 Then
    WScript.Echo "ERR|SOLIDWORKS export to STL failed. SaveErrors=" & saveErrors & "; Warnings=" & saveWarnings & "; COM=" & Err.Number
    swApp.CloseDoc swModel.GetTitle
    If startedByUs Then swApp.ExitApp
    WScript.Quit 12
End If

swApp.CloseDoc swModel.GetTitle
If startedByUs Then swApp.ExitApp

WScript.Echo "OK|" & loadErrors & "|" & loadWarnings
WScript.Quit 0
