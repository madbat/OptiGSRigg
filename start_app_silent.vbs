Set fso = CreateObject("Scripting.FileSystemObject")
Set sh = CreateObject("WScript.Shell")
appDir = fso.GetParentFolderName(WScript.ScriptFullName)
sh.CurrentDirectory = appDir
localVenvPy = appDir & "\.venv\Scripts\python.exe"
legacyVenvPy = "D:\OptiGSRigg\.venv\Scripts\python.exe"
If fso.FileExists(localVenvPy) Then
    cmd = """" & localVenvPy & """ """ & appDir & "\app.py"""
ElseIf fso.FileExists(legacyVenvPy) Then
    cmd = """" & legacyVenvPy & """ """ & appDir & "\app.py"""
Else
    cmd = "python """ & appDir & "\app.py"""
End If
sh.Run cmd, 0, False
