Option Explicit

Dim args, sourceFile, outputFile, sourceFolder, fileNameOnly, fileExt
Dim swApp, swModel, alreadyOpen
Dim docType, startedByUs, openedByBridge
Dim saveResult, saveErr, saveDescription
Dim attempts, stlCount
Dim oldStlBinary, oldStlShowInfo, oldStlDontTranslate, oldStlSingleFile
Dim oldStlPreview, oldStlInterference
Dim oldStlQuality, oldStlUnits
Dim fso, exportFolder, fileItem

Set args = WScript.Arguments
If args.Count < 2 Then
    WScript.Echo "ERR|Usage: solidworks_bridge.vbs source_file output_file"
    WScript.Quit 2
End If

sourceFile = args(0)
outputFile = args(1)
sourceFolder = Left(sourceFile, InStrRev(sourceFile, "\") - 1)
fileNameOnly = Mid(sourceFile, InStrRev(sourceFile, "\") + 1)
fileExt = LCase(Mid(fileNameOnly, InStrRev(fileNameOnly, ".")))
startedByUs = False
openedByBridge = False

Set fso = CreateObject("Scripting.FileSystemObject")

If fileExt = ".sldprt" Then
    docType = 1
ElseIf fileExt = ".sldasm" Then
    docType = 2
Else
    WScript.Echo "ERR|Unsupported SolidWorks file type: " & fileExt
    WScript.Quit 3
End If

WScript.Echo "BRIDGE_START|source=" & sourceFile & "|output=" & outputFile & "|docType=" & docType

On Error Resume Next

Err.Clear
Set swApp = GetObject("", "SldWorks.Application")
If Err.Number <> 0 Or swApp Is Nothing Then
    Err.Clear
    Set swApp = CreateObject("SldWorks.Application")
    startedByUs = True
End If

If Err.Number <> 0 Or swApp Is Nothing Then
    saveErr = Err.Number
    saveDescription = Err.Description
    WScript.Echo "ERR|Cannot connect to SOLIDWORKS. COM=" & saveErr & "|" & saveDescription
    WScript.Quit 10
End If

Err.Clear
swApp.Visible = True
WScript.Echo "BRIDGE_SOLIDWORKS_APP|startedByUs=" & startedByUs
WScript.Sleep 2500

Err.Clear
swApp.SetCurrentWorkingDirectory sourceFolder
WScript.Echo "BRIDGE_WORKING_DIRECTORY|path=" & sourceFolder & "|COM=" & Err.Number

' Reuse the exact document if it is already open.
Err.Clear
Set alreadyOpen = swApp.GetOpenDocumentByName(sourceFile)
If Not alreadyOpen Is Nothing Then
    Set swModel = alreadyOpen
    WScript.Echo "BRIDGE_ALREADY_OPEN|title=" & swModel.GetTitle
End If

' Use the simple legacy OpenDoc call instead of OpenDoc7. This is the
' established out-of-process automation path for local SolidWorks installs.
If swModel Is Nothing Then
    Err.Clear
    WScript.Echo "BRIDGE_OPENDOC|source=" & sourceFile & "|docType=" & docType
    Set swModel = swApp.OpenDoc(sourceFile, docType)
    saveErr = Err.Number
    saveDescription = Err.Description
    WScript.Echo "BRIDGE_OPENDOC_RESULT|model=" & Not (swModel Is Nothing) & "|COM=" & saveErr & "|" & saveDescription
    Err.Clear
    If Not swModel Is Nothing Then openedByBridge = True
End If

' Final fallback through the native Windows file association.
If swModel Is Nothing Then
    Err.Clear
    WScript.Echo "BRIDGE_SHELL_OPEN|source=" & sourceFile
    CreateObject("Shell.Application").Open sourceFile
    attempts = 0
    Do While attempts < 90 And swModel Is Nothing
        WScript.Sleep 1000
        Err.Clear
        Set swModel = swApp.GetOpenDocumentByName(sourceFile)
        If swModel Is Nothing Then
            Err.Clear
            Set swModel = swApp.GetOpenDocumentByName(fileNameOnly)
        End If
        attempts = attempts + 1
    Loop
    If Not swModel Is Nothing Then openedByBridge = True
    WScript.Echo "BRIDGE_SHELL_OPEN_RESULT|model=" & Not (swModel Is Nothing) & "|attempts=" & attempts & "|COM=" & Err.Number
End If

If swModel Is Nothing Then
    saveErr = Err.Number
    saveDescription = Err.Description
    WScript.Echo "ERR|SOLIDWORKS returned no document object. COM=" & saveErr & "|" & saveDescription
    If startedByUs Then swApp.ExitApp
    WScript.Quit 12
End If

Err.Clear
WScript.Echo "BRIDGE_DOCUMENT_READY|title=" & swModel.GetTitle & "|openedByBridge=" & openedByBridge

' Disable modal STL dialogs and force one combined assembly STL while the
' export runs. The preferences are restored immediately afterwards.
oldStlBinary = swApp.GetUserPreferenceToggle(69)
oldStlShowInfo = swApp.GetUserPreferenceToggle(70)
oldStlDontTranslate = swApp.GetUserPreferenceToggle(71)
oldStlSingleFile = swApp.GetUserPreferenceToggle(72)
oldStlInterference = swApp.GetUserPreferenceToggle(73)
oldStlPreview = swApp.GetUserPreferenceToggle(191)
oldStlQuality = swApp.GetUserPreferenceIntegerValue(78)
oldStlUnits = swApp.GetUserPreferenceIntegerValue(211)

WScript.Echo "BRIDGE_STL_PREFS_BEFORE|binary=" & oldStlBinary & "|showInfo=" & oldStlShowInfo & "|dontTranslate=" & oldStlDontTranslate & "|singleFile=" & oldStlSingleFile & "|interference=" & oldStlInterference & "|preview=" & oldStlPreview & "|quality=" & oldStlQuality & "|units=" & oldStlUnits & "|COM=" & Err.Number

Err.Clear
swApp.SetUserPreferenceToggle 69, True
swApp.SetUserPreferenceToggle 70, False
swApp.SetUserPreferenceToggle 71, True
swApp.SetUserPreferenceToggle 72, True
swApp.SetUserPreferenceToggle 73, False
swApp.SetUserPreferenceToggle 191, False
swApp.SetUserPreferenceIntegerValue 78, 2
swApp.SetUserPreferenceIntegerValue 211, 0
WScript.Echo "BRIDGE_STL_PREFS_SET|binary=True|showInfo=False|dontTranslate=True|singleFile=True|interference=False|preview=False|quality=Fine|units=mm|COM=" & Err.Number

Err.Clear
swModel.ClearSelection2 True
WScript.Echo "BRIDGE_SELECTION_CLEARED|COM=" & Err.Number

Err.Clear
WScript.Echo "BRIDGE_SAVEAS_LEGACY|starting|output=" & outputFile
saveResult = swModel.SaveAs(outputFile)
saveErr = Err.Number
saveDescription = Err.Description
WScript.Echo "BRIDGE_SAVEAS_LEGACY_RESULT|result=" & saveResult & "|COM=" & saveErr & "|" & saveDescription
Err.Clear

' Wait for filesystem completion. Large assemblies can finish asynchronously.
attempts = 0
Do While attempts < 60 And Not fso.FileExists(outputFile)
    WScript.Sleep 1000
    attempts = attempts + 1
Loop
WScript.Echo "BRIDGE_OUTPUT_WAIT|requestedExists=" & fso.FileExists(outputFile) & "|seconds=" & attempts

' SolidWorks can export an assembly as one STL per component even when the
' automation call returns True. Those component STLs are still valid geometry;
' the Python server will merge them into the single STL consumed by Three.js.
stlCount = 0
Err.Clear
Set exportFolder = fso.GetFolder(fso.GetParentFolderName(outputFile))
For Each fileItem In exportFolder.Files
    If LCase(fso.GetExtensionName(fileItem.Name)) = "stl" Then
        stlCount = stlCount + 1
    End If
Next
WScript.Echo "BRIDGE_STL_FILES_IN_EXPORT_DIR|count=" & stlCount

If Not fso.FileExists(outputFile) And stlCount = 0 Then
    WScript.Echo "ERR|SOLIDWORKS created no STL output."
    Err.Clear
    swApp.SetUserPreferenceToggle 69, oldStlBinary
    swApp.SetUserPreferenceToggle 70, oldStlShowInfo
    swApp.SetUserPreferenceToggle 71, oldStlDontTranslate
    swApp.SetUserPreferenceToggle 72, oldStlSingleFile
    swApp.SetUserPreferenceToggle 73, oldStlInterference
    swApp.SetUserPreferenceToggle 191, oldStlPreview
    swApp.SetUserPreferenceIntegerValue 78, oldStlQuality
    swApp.SetUserPreferenceIntegerValue 211, oldStlUnits
    If startedByUs Then
        swApp.CloseDoc swModel.GetTitle
        swApp.ExitApp
    End If
    WScript.Quit 15
End If

If fso.FileExists(outputFile) Then
    WScript.Echo "BRIDGE_STL_OUTPUT_READY|path=" & outputFile
Else
    WScript.Echo "BRIDGE_COMPONENT_STL_OUTPUTS|count=" & stlCount
End If

Err.Clear
swApp.SetUserPreferenceToggle 69, oldStlBinary
swApp.SetUserPreferenceToggle 70, oldStlShowInfo
swApp.SetUserPreferenceToggle 71, oldStlDontTranslate
swApp.SetUserPreferenceToggle 72, oldStlSingleFile
swApp.SetUserPreferenceToggle 73, oldStlInterference
swApp.SetUserPreferenceToggle 191, oldStlPreview
swApp.SetUserPreferenceIntegerValue 78, oldStlQuality
swApp.SetUserPreferenceIntegerValue 211, oldStlUnits
WScript.Echo "BRIDGE_STL_PREFS_RESTORE|COM=" & Err.Number

WScript.Echo "BRIDGE_DONE"
WScript.Echo "OK|0|0"

If startedByUs Then
    Err.Clear
    swApp.CloseDoc swModel.GetTitle
    swApp.ExitApp
End If

WScript.Quit 0
