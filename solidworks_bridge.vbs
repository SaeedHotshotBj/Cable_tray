Option Explicit

Dim args, sourceFile, outputFile, sourceFolder, fileNameOnly
Dim swApp, swModel, swActiveModel, swDocSpec, alreadyOpen
Dim loadErrors, loadWarnings, saveErrors, saveWarnings, result
Dim docType, startedByUs, attempts, activateErrors, activationAttempts, targetTitle
Dim oldStlSingleFile, oldStlBinary, oldStlQuality, oldStlUnits, stlPrefOk
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
targetTitle = swModel.GetTitle
WScript.Echo "BRIDGE_TARGET_DOCUMENT|title=" & targetTitle

' ActivateDoc3 itself returns the ModelDoc2/ModelDoc object that it activated.
' Use that return value. SOLIDWORKS may transiently return Nothing from the
' separate ActiveDoc property while COM activation is still settling.
activateErrors = 0
activationAttempts = 0
Set swActiveModel = Nothing

Do While activationAttempts < 10 And swActiveModel Is Nothing
    activationAttempts = activationAttempts + 1

    Err.Clear
    activateErrors = 0
    WScript.Echo "BRIDGE_ACTIVATE|attempt=" & activationAttempts & "|name=" & targetTitle
    Set swActiveModel = swApp.ActivateDoc3(targetTitle, False, 0, activateErrors)
    WScript.Echo "BRIDGE_ACTIVATE_RESULT|model=" & Not (swActiveModel Is Nothing) & "|errors=" & activateErrors & "|COM=" & Err.Number

    ' swGenericActivateError = 1 means the document was not activated.
    ' swDocNeedsRebuildWarning = 2 is a warning and the document remains usable.
    If Not swActiveModel Is Nothing And activateErrors <> 0 And activateErrors <> 2 Then
        Set swActiveModel = Nothing
    End If

    ' Retry by fully-qualified source path if activation by title did not return
    ' the document object. Do not overwrite swModel here.
    If swActiveModel Is Nothing Then
        Err.Clear
        activateErrors = 0
        WScript.Echo "BRIDGE_ACTIVATE_PATH_FALLBACK|attempt=" & activationAttempts
        Set swActiveModel = swApp.ActivateDoc3(sourceFile, False, 0, activateErrors)
        WScript.Echo "BRIDGE_ACTIVATE_PATH_FALLBACK_RESULT|model=" & Not (swActiveModel Is Nothing) & "|errors=" & activateErrors & "|COM=" & Err.Number

        If Not swActiveModel Is Nothing And activateErrors <> 0 And activateErrors <> 2 Then
            Set swActiveModel = Nothing
        End If
    End If

    If swActiveModel Is Nothing Then WScript.Sleep 1000
Loop

If swActiveModel Is Nothing Then
    ' Final compatibility check. Some SOLIDWORKS versions expose ActiveDoc only
    ' after activation has settled, even when ActivateDoc3 did not return it.
    Err.Clear
    Set swActiveModel = swApp.ActiveDoc
    WScript.Echo "BRIDGE_ACTIVE_DOC_FINAL_CHECK|model=" & Not (swActiveModel Is Nothing) & "|COM=" & Err.Number
End If

If swActiveModel Is Nothing Then
    comError = Err.Number
    comDescription = Err.Description
    WScript.Echo "ERR|Active document object unavailable after activation. COM=" & comError & "|" & comDescription
    swApp.CloseDoc targetTitle
    If startedByUs Then swApp.ExitApp
    WScript.Quit 13
End If

Err.Clear
WScript.Echo "BRIDGE_ACTIVE_DOC|title=" & swActiveModel.GetTitle

Err.Clear
swActiveModel.ClearSelection2 True
If docType = 2 Then
    swActiveModel.ResolveAllLightWeightComponents False
    swActiveModel.ForceRebuild3 False
    WScript.Echo "BRIDGE_REBUILD|resolved_components_requested"
    WScript.Sleep 1000
End If

' Force deterministic STL export settings for the Cable_tray importer.
' In particular, assemblies must be written to ONE STL file. Otherwise
' SOLIDWORKS can legitimately report a successful conversion while creating
' component STL files instead of the requested assembly output path.
Err.Clear
oldStlSingleFile = swApp.GetUserPreferenceToggle(72) ' swSTLComponentsIntoOneFile
oldStlBinary = swApp.GetUserPreferenceToggle(69)     ' swSTLBinaryFormat
oldStlQuality = swApp.GetUserPreferenceIntegerValue(78) ' swExportSTLQuality
oldStlUnits = swApp.GetUserPreferenceIntegerValue(211)   ' swExportStlUnits
WScript.Echo "BRIDGE_STL_PREFS_BEFORE|singleFile=" & oldStlSingleFile & "|binary=" & oldStlBinary & "|quality=" & oldStlQuality & "|units=" & oldStlUnits & "|COM=" & Err.Number

Err.Clear
stlPrefOk = swApp.SetUserPreferenceToggle(72, True)
WScript.Echo "BRIDGE_STL_PREF_SINGLE_FILE|result=" & stlPrefOk & "|value=True|COM=" & Err.Number

Err.Clear
swApp.SetUserPreferenceToggle 69, True
swApp.SetUserPreferenceIntegerValue 78, 2
swApp.SetUserPreferenceIntegerValue 211, 0
WScript.Echo "BRIDGE_STL_PREFS_SET|binary=True|quality=Fine|units=mm|COM=" & Err.Number

saveErrors = 0
saveWarnings = 0
Err.Clear
WScript.Echo "BRIDGE_SAVEAS3|starting|output=" & outputFile
result = swActiveModel.Extension.SaveAs3(outputFile, 0, 1, Nothing, Nothing, saveErrors, saveWarnings)
WScript.Echo "BRIDGE_SAVEAS3_RESULT|result=" & result & "|saveErrors=" & saveErrors & "|saveWarnings=" & saveWarnings & "|COM=" & Err.Number

If result <> True Or saveErrors <> 0 Then
    comError = Err.Number
    comDescription = Err.Description
    WScript.Echo "BRIDGE_SAVEAS3_FAILED|saveErrors=" & saveErrors & ";Warnings=" & saveWarnings & ";COM=" & comError & "|" & comDescription
Else
    ' SaveAs3 can return success before Windows has finished materializing the
    ' output file. Poll for up to 30 seconds instead of assuming 2 seconds is
    ' always enough for a large assembly.
    attempts = 0
    Do While attempts < 30 And Not CreateObject("Scripting.FileSystemObject").FileExists(outputFile)
        WScript.Sleep 1000
        attempts = attempts + 1
    Loop
    WScript.Echo "BRIDGE_STL_FILE_WAIT|exists=" & CreateObject("Scripting.FileSystemObject").FileExists(outputFile) & "|seconds=" & attempts
End If

If Not CreateObject("Scripting.FileSystemObject").FileExists(outputFile) Then
    ' Compatibility fallback: SaveAs4 on ModelDoc2 uses the older, simpler
    ' export entry point and is still present in current SOLIDWORKS versions.
    saveErrors = 0
    saveWarnings = 0
    Err.Clear
    WScript.Echo "BRIDGE_SAVEAS4_FALLBACK|starting|output=" & outputFile
    result = swActiveModel.SaveAs4(outputFile, 0, 1, saveErrors, saveWarnings)
    WScript.Echo "BRIDGE_SAVEAS4_RESULT|result=" & result & "|saveErrors=" & saveErrors & "|saveWarnings=" & saveWarnings & "|COM=" & Err.Number

    attempts = 0
    Do While attempts < 30 And Not CreateObject("Scripting.FileSystemObject").FileExists(outputFile)
        WScript.Sleep 1000
        attempts = attempts + 1
    Loop
    WScript.Echo "BRIDGE_STL_FILE_WAIT_AFTER_SAVEAS4|exists=" & CreateObject("Scripting.FileSystemObject").FileExists(outputFile) & "|seconds=" & attempts
End If

If Not CreateObject("Scripting.FileSystemObject").FileExists(outputFile) Then
    WScript.Echo "ERR|SOLIDWORKS did not create the requested STL output after SaveAs3 and SaveAs4."
    Err.Clear
    swApp.SetUserPreferenceToggle 72, oldStlSingleFile
    swApp.SetUserPreferenceToggle 69, oldStlBinary
    swApp.SetUserPreferenceIntegerValue 78, oldStlQuality
    swApp.SetUserPreferenceIntegerValue 211, oldStlUnits
    swApp.CloseDoc swModel.GetTitle
    If startedByUs Then swApp.ExitApp
    WScript.Quit 15
End If

WScript.Echo "BRIDGE_STL_OUTPUT_READY|path=" & outputFile
WScript.Echo "BRIDGE_STL_PREFS_RESTORE|starting"
Err.Clear
swApp.SetUserPreferenceToggle 72, oldStlSingleFile
swApp.SetUserPreferenceToggle 69, oldStlBinary
swApp.SetUserPreferenceIntegerValue 78, oldStlQuality
swApp.SetUserPreferenceIntegerValue 211, oldStlUnits
WScript.Echo "BRIDGE_STL_PREFS_RESTORE|done|COM=" & Err.Number

WScript.Echo "BRIDGE_DONE"
WScript.Echo "OK|0|0"

swApp.CloseDoc swModel.GetTitle
If startedByUs Then swApp.ExitApp
WScript.Quit 0
