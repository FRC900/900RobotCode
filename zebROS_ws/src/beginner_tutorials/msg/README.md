To add a new message:
- Put a new .msg file in this folder
- Add your message file (e.g. YourMessage.msg) to CMakeLists.txt like this:
```
add_message_files(
  FILES
  ZebraData.msg
  YourMessage.msg
)
```