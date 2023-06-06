/^# Packages using this file: / {
  s/# Packages using this file://
  ta
  :a
  s/ eegdev / eegdev /
  tb
  s/ $/ eegdev /
  :b
  s/^/# Packages using this file:/
}
