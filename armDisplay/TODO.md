# armDisplay TODO

## High Priority
- [x] Fix memory leak: `noise_overlay()` creates new canvas items every 100ms without deleting old ones — track the item ID and delete before recreating
- [x] Fix `screen_flicker()`: flashing canvas bg to bright green is a photosensitivity hazard — either remove or reduce to subtle brightness variation
- [x] Add error handling for missing `background.jpeg` — fallback to solid black or pattern
- [x] Implement BLE client in `button_action()` to send commands to other props

## Medium Priority
- [x] Optimize vignette computation: 360 ellipse draws block the main thread at startup — pre-render to a static image or reduce resolution
- [ ] Theme button labels to match Tech Priest aesthetic (e.g., "PURGE", "SCOUT", "EXorcise", "RECALIBRATE")
- [x] Add graceful shutdown: bind `<Escape>` or SIGINT to clean up `after()` loops and exit cleanly
- [x] Store width/height as instance variables instead of passing them around

## Low Priority
- [ ] Consider using `tkinter.after` with a single update loop instead of four independent timers for better frame synchronization
- [ ] Add configuration file for button labels, colors, and effect timing
- [x] Add logging instead of `print()` in `button_action()`

## BLE Client (PR #14)
- [x] Add bleak dependency with graceful fallback
- [x] Implement BLE_Controller class (connect, disconnect, send_command)
- [x] Add device discovery by name (targets `Plasma_Pistol`)
- [x] Add BLE status indicator label
- [x] Run asyncio event loop in background thread
- [ ] Add reconnection logic on disconnect
- [ ] Add support for multiple target props (LEDGoggles, servoSkull)
