PROJECT (Firmware LANGUAGES CSharp)
CMAKE_MINIMUM_REQUIRED ( VERSION 3.12)
set(CMAKE_DOTNET_TARGET_FRAMEWORK_VERSION "4.5.2")
add_library (Firmware SHARED Firmware.cs CommandType.cs App.config Properties/AssemblyInfo.cs)

add_custom_command(TARGET ${PROJECT_NAME} 
	PRE_BUILD
	COMMAND nuget install CPTR302VictoriaPohleHardware -version 1.0.0 -OutputDirectory ${CMAKE_BINARY_DIR}
	)

set_property(TARGET Firmware PROPERTY VS_DOTNET_REFERENCES  "CPTR302VictoriaPohleHardware.1.0.0/lib/net452/CPTR302VictoriaPohleHardware.dll" )

set(CMAKE_CSharp_FLAGS "/langversion:7")

 

