# armDisplay TODO

## High Priority
- [ ] Fix memory leak: `noise_overlay()` creates new canvas items every 100ms without deleting old ones — track the item ID and delete before recreating
- [ ] Fix `screen_flicker()`: flashing canvas bg to bright green is a photosensitivity hazard — either remove or reduce to subtle brightness variation
- [ ] Add error handling for missing `background.jpeg` — fallback to solid black or pattern
- [ ] Implement BLE client in `button_action()` to send commands to other props

## Medium Priority
- [ ] Optimize vignette computation: 360 ellipse draws block the main thread at startup — pre-render to a static image or reduce resolution
- [ ] Theme button labels to match Tech Priest aesthetic (e.g., "PURGE", "SCOUT", "EXorcise", "RECALIBRATE")
- [ ] Add graceful shutdown: bind `<Escape>` or SIGINT to clean up `after()` loops and exit cleanly
- [ ] Store width/height as instance variables instead of passing them around

## Low Priority
- [ ] Consider using `tkinter.after` with a single update loop instead of four independent timers for better frame synchronization
- [ ] Add configuration file for button labels, colors, and effect timing
- [ ] Add logging instead of `print()` in `button_action()`
