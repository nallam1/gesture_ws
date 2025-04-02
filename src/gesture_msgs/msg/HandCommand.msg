# Hand being controlled
string hand  # "left" or "right"

# Gesture classification result
string gesture  # e.g., "open_hand", "closed_fist", "pronation", etc.

# Motion control parameters
float32 flex_ext   # Ranging: -90degrees (flexion), +90degrees (extension)
#float32 deviation   # Ranging: -28degrees (ulnar deviation), +38degrees (radial deviation), etc.
float32 pron_supi    # Ranging: -90degrees (pronation), +90degrees (supination) 
float32 grip_state  # Ranging: 0.0 (open), 0.5 (relaxed), 1.0 (closed)

