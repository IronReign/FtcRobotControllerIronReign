assets/configs can be used to store robot config xml files that need to end up in  /sdcard/FIRST

Config files stored here are under source control and bypass xml parsing during build

This allows files with badly formed xml (like those that include a Limelight config) to
not break the build and still be selectable by the driver station.

These files are also editable in the driver station config app, but will be overwritten in subsequent builds.

The files are transported after build with custom gradle tasks defined in Teamcode/build.gradle