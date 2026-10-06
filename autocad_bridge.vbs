Option Explicit

Dim args, sourceFile, outputFile, sourceExt
Dim acadApp, acadDoc, docItem
Dim startedByUs, openedByBridge
Dim fso, attempts, fileDated, oldFileDia, oldFacetres
Dim insertPoint(2)
Dim commandText, saveErr, saveDescription

Set args = WScript.Arguments
If args.Count < 2 Then
    WScript.Echo "ERR|Usage: autocad_bridge.vbs source_file output_file"
    WScript.Quit 2
End If

sourceFile = args(0)
outputFile = args(1)
sourceExt = LCase(Mid(sourceFile, InStrRev(sourceFile, ".")))
startedByUs = False
openedByBridge = False
Set fso = CreateObject("Scripting.FileSystemObject")

If sourceExt <> ".dwg" And sourceExt <> ".dxf" Then
    WScript.Echo "ERR|Unsupported AutoCAD file type: " & sourceExt
    WScript.Quit 3
End If

WScript.Echo "BRIDGE_START|source=" & sourceFile & "|output=" & outputFile

On Error Resume Next

Err.Clear
Set acadApp = GetObject(, "AutoCAD.Application")
If Err.Number <> 0 Or acadApp Is Nothing Then
    Err.Clear
    Set acadApp = CreateObject("AutoCAD.Application")
    startedByUs = True
End If

If Err.Number <> 0 Or acadApp Is Nothing Then
    saveErr = Err.Number
    saveDescription = Err.Description
    WScript.Echo "ERR|Cannot connect to AutoCAD. COM=" & saveErr & "|" & saveDescription
    WScript.Quit 10
End If

Err.Clear
acadApp.Visible = True
WScript.Echo "BRIDGE_AUTOCAD_APP|startedByUs=" & startedByUs & "|version=" & acadApp.Version
WScript.Sleep 2500

' Reuse the exact drawing when it is already open.
Err.Clear
Set acadDoc = Nothing
For Each docItem In acadApp.Documents
    If LCase(docItem.FullName) = LCase(sourceFile) Then
        Set acadDoc = docItem
        Exit For
    End If
Next

If Not acadDoc Is Nothing Then
    openedByBridge = False
    WScript.Echo "BRIDGE_ALREADY_OPEN|name=" & acadDoc.Name
End If

' DWG is opened through the Documents collection. DXF is imported into
' a temporary drawing because AutoCAD documents.Open is for DWG files,
' while the ActiveX Import method explicitly supports DXF.
If acadDoc Is Nothing And sourceExt = ".dwg" Then
    Err.Clear
    WScript.Echo "BRIDGE_OPEN|source=" & sourceFile
    Set acadDoc = acadApp.Documents.Open(sourceFile, False)
    saveErr = Err.Number
    saveDescription = Err.Description
    WScript.Echo "BRIDGE_OPEN_RESULT|document=" & Not (acadDoc Is Nothing) & "|COM=" & saveErr & "|" & saveDescription
    Err.Clear
    If Not acadDoc Is Nothing Then openedByBridge = True
End If

If acadDoc Is Nothing And sourceExt = ".dxf" Then
    Err.Clear
    WScript.Echo "BRIDGE_DXF_IMPORT|source=" & sourceFile
    Set acadDoc = acadApp.Documents.Add("")
    insertPoint(0) = 0#
    insertPoint(1) = 0#
    insertPoint(2) = 0#
    Err.Clear
    acadDoc.Import sourceFile, insertPoint, 1#
    saveErr = Err.Number
    saveDescription = Err.Description
    WScript.Echo "BRIDGE_DXF_IMPORT_RESULT|document=" & Not (acadDoc Is Nothing) & "|COM=" & saveErr & "|" & saveDescription
    Err.Clear
    If Not acadDoc Is Nothing Then openedByBridge = True
End If

If acadDoc Is Nothing Then
    Err.Clear
    WScript.Echo "BRIDGE_SHELL_OPEN|source=" & sourceFile
    CreateObject("Shell.Application").Open sourceFile
    attempts = 0
    Do While attempts < 90 And acadDoc Is Nothing
        WScript.Sleep 1000
        Err.Clear
        Set acadDoc = acadApp.ActiveDocument
        If Not acadDoc Is Nothing Then
            If LCase(acadDoc.FullName) <> LCase(sourceFile) Then
                Set acadDoc = Nothing
            End If
        End If
        attempts = attempts + 1
    Loop
    If Not acadDoc Is Nothing Then openedByBridge = True
    WScript.Echo "BRIDGE_SHELL_OPEN_RESULT|document=" & Not (acadDoc Is Nothing) & "|attempts=" & attempts & "|COM=" & Err.Number
End If

If acadDoc Is Nothing Then
    saveErr = Err.Number
    saveDescription = Err.Description
    WScript.Echo "ERR|AutoCAD returned no document object. COM=" & saveErr & "|" & saveDescription
    If startedByUs Then acadApp.Quit
    WScript.Quit 12
End If

Err.Clear
acadDoc.Activate
WScript.Echo "BRIDGE_DOCUMENT_READY|name=" & acadDoc.Name & "|openedByBridge=" & openedByBridge & "|COM=" & Err.Number

' Suppress file dialogs and export all 3D solids/watertight meshes as one STL.
' AutoCAD documents STLOUT for 3D solids and watertight meshes; FILEDIA=0
' allows the output filename to be supplied on the command line.
oldFileDia = acadDoc.GetVariable("FILEDIA")
oldFacetres = acadDoc.GetVariable("FACETRES")

Err.Clear
acadDoc.SetVariable "FILEDIA", 0
acadDoc.SetVariable "FACETRES", 2.0
WScript.Echo "BRIDGE_STL_PREFS_SET|FILEDIA=0|FACETRES=2|COM=" & Err.Number

Err.Clear
acadDoc.Regen 0
WScript.Echo "BRIDGE_REGEN|COM=" & Err.Number

If fso.FileExists(outputFile) Then
    fso.DeleteFile outputFile, True
End If

' STLOUT command line sequence:
'   selection = ALL
'   binary STL = Yes
'   output filename = supplied path
commandText = "_.-STLOUT _ALL _Y " & Chr(34) & outputFile & Chr(34) & " "
WScript.Echo "BRIDGE_STLOUT|command=" & commandText

Err.Clear
acadDoc.SendCommand commandText
saveErr = Err.Number
saveDescription = Err.Description
WScript.Echo "BRIDGE_STLOUT_SENT|COM=" & saveErr & "|" & saveDescription
Err.Clear

attempts = 0
Do While attempts < 120 And Not fso.FileExists(outputFile)
    WScript.Sleep 1000
    attempts = attempts + 1
Loop

WScript.Echo "BRIDGE_OUTPUT_WAIT|exists=" & fso.FileExists(outputFile) & "|seconds=" & attempts

' Restore AutoCAD settings even when export failed.
Err.Clear
acadDoc.SetVariable "FILEDIA", oldFileDia
acadDoc.SetVariable "FACETRES", oldFacetres
WScript.Echo "BRIDGE_STL_PREFS_RESTORE|COM=" & Err.Number

If Not fso.FileExists(outputFile) Then
    WScript.Echo "ERR|AutoCAD did not create an STL. The DWG/DXF may contain no 3D solids or watertight meshes, or AutoCAD may be waiting on a command."
    If startedByUs Then
        Err.Clear
        acadDoc.Close False
        acadApp.Quit
    End If
    WScript.Quit 15
End If

WScript.Echo "BRIDGE_STL_OUTPUT_READY|path=" & outputFile
WScript.Echo "BRIDGE_DONE"
WScript.Echo "OK|0|0"

If startedByUs Then
    Err.Clear
    acadDoc.Close False
    acadApp.Quit
End If

WScript.Quit 0
