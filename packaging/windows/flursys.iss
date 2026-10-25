#ifndef AppVersion
  #error AppVersion must be supplied
#endif
#ifndef StageDir
  #error StageDir must be supplied
#endif
#ifndef OutputDir
  #error OutputDir must be supplied
#endif
#ifndef OutputName
  #error OutputName must be supplied
#endif

#ifndef FileVersion
  #define FileVersion AppVersion
#endif

[Setup]
AppId={{36A5BFC8-650B-47C0-BDCA-E5118C311148}
AppName=FLURSYS
AppVersion={#AppVersion}
VersionInfoVersion={#FileVersion}
AppPublisher=FLURSYS contributors
AppPublisherURL=https://github.com/A1i98/FLURSYS
DefaultDirName={localappdata}\Programs\FLURSYS
DefaultGroupName=FLURSYS
PrivilegesRequired=lowest
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
MinVersion=10.0
DisableProgramGroupPage=yes
LicenseFile={#StageDir}\LICENSE
InfoBeforeFile={#StageDir}\BUNDLED-TOOLS.txt
SetupIconFile={#SourcePath}\flursys.ico
UninstallDisplayIcon={app}\flursys-gui.exe
OutputDir={#OutputDir}
OutputBaseFilename={#OutputName}
Compression=lzma2/fast
SolidCompression=yes
WizardStyle=modern
CloseApplications=yes
RestartApplications=no

[Tasks]
Name: desktopicon; Description: "Create a desktop shortcut"; GroupDescription: "Shortcuts:"; Flags: unchecked

[Files]
Source: "{#StageDir}\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs

[Icons]
Name: "{group}\FLURSYS"; Filename: "{app}\flursys-gui.exe"; WorkingDir: "{app}"
Name: "{group}\Gmsh (bundled)"; Filename: "{app}\gmsh\gmsh.exe"; WorkingDir: "{app}\gmsh"
Name: "{autodesktop}\FLURSYS"; Filename: "{app}\flursys-gui.exe"; WorkingDir: "{app}"; Tasks: desktopicon

[Run]
Filename: "{app}\flursys-gui.exe"; Description: "Launch FLURSYS"; Flags: nowait postinstall skipifsilent
