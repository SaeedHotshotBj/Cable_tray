Option Explicit

Dim args, sourceFile, outputFile, sourceFolder, fileNameOnly
Dim swApp, swModel, swActiveModel, swDocSpec, alreadyOpen
Dim loadErrors, loadWarnings, saveErrors, saveWarnings, result
Dim docType, startedByUs, attempts
Dim specError, specWarning, comError, comDescription

Set args = WScript.Arguments
If args.Count < 2 Then
    WScript.Echo "ERR|Usage: solidworks_bridge.vbs source_file output_file"
    WScript.Quit 2
End If

sourceFile = args(0)
outputFile = args(1)
sourceFolder = Left(sourceFile, InStrRev(sourceFile, "\") - 1)
fileNameOnly = Mid(sourceFile, InStrRev(sourceFile, "\") + 1)
startedByUs = False

WScript.Echo "BRIDGE_START|source=" & sourceFile & "|output=" & outputFile

On Error Resume Next

Err.Clear
Set swApp = GetObject("", "SldWorks.Application")
If Err.Number <> 0 Or swApp Is Nothing Then
    Err.Clear
    Set swApp = CreateObject("SldWorks.Application")
    startedByUs = True
End If

If Err.Number <> 0 Or swApp Is Nothing Then
    comError = Err.Number
    comDescription = Err.Description
    WScript.Echo "ERR|Cannot connect to SOLIDWORKS. COM=" & comError & "|" & comDescription
    WScript.Quit 10
End If

Err.Clear
swApp.Visible = True
WScript.Echo "BRIDGE_SOLIDWORKS_APP|startedByUs=" & startedByUs
WScript.Sleep 4000

Err.Clear
result = swApp.SetCurrentWorkingDirectory(sourceFolder)
WScript.Echo "BRIDGE_WORKING_DIRECTORY|path=" & sourceFolder & "|result=" & result & "|COM=" & Err.Number

docType = 2

' Reuse the document if SOLIDWORKS already has this assembly open.
Err.Clear
Set alreadyOpen = swApp.GetOpenDocumentByName(sourceFile)
If Not alreadyOpen Is Nothing Then
    Set swModel = alreadyOpen
    WScript.Echo "BRIDGE_ALREADY_OPEN|title=" & swModel.GetTitle
End If

If swModel Is Nothing Then
    Err.Clear
    Set swDocSpec = swApp.GetOpenDocSpec(sourceFile)
    If swDocSpec Is Nothing Or Err.Number <> 0 Then
        comError = Err.Number
        comDescription = Err.Description
        WScript.Echo "ERR|GetOpenDocSpec failed. COM=" & comError & "|" & comDescription
        If startedByUs Then swApp.ExitApp
        WScript.Quit 11
    End If

    swDocSpec.DocumentType = docType
    swDocSpec.FileName = sourceFile
    swDocSpec.ConfigurationName = ""
    swDocSpec.Silent = False
    swDocSpec.UseLightWeightDefault = False
    swDocSpec.LightWeight = False
    swDocSpec.IgnoreHiddenComponents = False
    swDocSpec.LoadModel = True

    WScript.Echo "BRIDGE_OPENDOC7|documentType=" & docType & "|silent=" & swDocSpec.Silent & "|lightweight=" & swDocSpec.LightWeight

    Err.Clear
    Set swModel = swApp.OpenDoc7(swDocSpec)
    specError = swDocSpec.Error
    specWarning = swDocSpec.Warning

    WScript.Echo "BRIDGE_OPENDOC7_RESULT|model=" & Not (swModel Is Nothing) & "|error=" & specError & "|warning=" & specWarning & "|COM=" & Err.Number

    If swModel Is Nothing Or specError <> 0 Then
        ' Fallback 1: OpenDoc6 directly.
    loadErrors = 0
    loadWarnings = 0
    Err.Clear
    WScript.Echo "BRIDGE_OPENDOC6_FALLBACK|starting"
    Set swModel = swApp.OpenDoc6(sourceFile, docType, 0, "", loadErrors, loadWarnings)
    WScript.Echo "BRIDGE_OPENDOC6_FALLBACK_RESULT|model=" & Not (swModel Is Nothing) & "|loadErrors=" & loadErrors & "|loadWarnings=" & loadWarnings & "|COM=" & Err.Number

    ' Fallback 2: use the native Windows/SOLIDWORKS file association.
    ' This is equivalent to opening the SLDASM from Explorer and lets
    ' SOLIDWORKS resolve assembly references through its normal UI path.
    If swModel Is Nothing Then
        Err.Clear
        WScript.Echo "BRIDGE_SHELL_OPEN|source=" & sourceFile
        CreateObject("Shell.Application").Open sourceFile
        WScript.Sleep 5000

        attempts = 0
        Do While attempts < 60 And swModel Is Nothing
            Err.Clear
            Set swModel = swApp.GetOpenDocumentByName(sourceFile)
            If swModel Is Nothing Then
                Err.Clear
                Set swModel = swApp.GetOpenDocumentByName(fileNameOnly)
            End If
            If swModel Is Nothing Then WScript.Sleep 1000
            attempts = attempts + 1
        Loop
        WScript.Echo "BRIDGE_SHELL_OPEN_RESULT|model=" & Not (swModel Is Nothing) & "|attempts=" & attempts & "|COM=" & Err.Number
    End If
    End If
End If

If swModel Is Nothing Then
    comError = Err.Number
    comDescription = Err.Description
    WScript.Echo "ERR|SOLIDWORKS returned no document object. COM=" & comError & "|" & comDescription
    If startedByUs Then swApp.ExitApp
    WScript.Quit 12
End If

WScript.Sleep 1500
Err.Clear
swApp.ActivateDoc3 swModel.GetTitle, True, 0
WScript.Sleep 1000

Err.Clear
Set swActiveModel = swApp.ActiveDoc
If swActiveModel Is Nothing Or Err.Number <> 0 Then
    comError = Err.Number
    comDescription = Err.Description
    WScript.Echo "ERR|ActiveDoc unavailable. COM=" & comError & "|" & comDescription
    swApp.CloseDoc swModel.GetTitle
    If startedByUs Then swApp.ExitApp
    WScript.Quit 13
End If

WScript.Echo "BRIDGE_ACTIVE_DOC|title=" & swActiveModel.GetTitle

Err.Clear
swActiveModel.ClearSelection2 True
If docType = 2 Then
    swActiveModel.ResolveAllLightWeightComponents False
    swActiveModel.ForceRebuild3 False
    WScript.Echo "BRIDGE_REBUILD|resolved_components_requested"
    WScript.Sleep 1000
End If

saveErrors = 0
saveWarnings = 0
Err.Clear
WScript.Echo "BRIDGE_SAVEAS3|starting|output=" & outputFile
result = swActiveModel.Extension.SaveAs3(outputFile, 0, 1, Nothing, Nothing, saveErrors, saveWarnings)
WScript.Echo "BRIDGE_SAVEAS3_RESULT|result=" & result & "|saveErrors=" & saveErrors & "|saveWarnings=" & saveWarnings & "|COM=" & Err.Number

If result <> True Or Err.Number <> 0 Then
    comError = Err.Number
    comDescription = Err.Description
    WScript.Echo "ERR|SOLIDWORKS STL export failed. SaveErrors=" & saveErrors & "; Warnings=" & saveWarnings & "; COM=" & comError & "|" & comDescription
    swApp.CloseDoc swModel.GetTitle
    If startedByUs Then swApp.ExitApp
    WScript.Quit 14
End If

If Not CreateObject("Scripting.FileSystemObject").FileExists(outputFile) Then
    WScript.Echo "ERR|SOLIDWORKS reported success but STL file was not created."
    swApp.CloseDoc swModel.GetTitle
    If startedByUs Then swApp.ExitApp
    WScript.Quit 15
End If

WScript.Echo "BRIDGE_DONE"
WScript.Echo "OK|0|0"

swApp.CloseDoc swModel.GetTitle
If startedByUs Then swApp.ExitApp
WScript.Quit 0
