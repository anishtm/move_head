string face_detected 
uint8 index         # Index of the detected face
uint8 box_left      # Left coordinate of the bounding box
uint8 box_top       # Top coordinate of the bounding box
uint8 box_right     # Right coordinate of the bounding box
uint8 box_bottom    # Bottom coordinate of the bounding box
int8 box_confidence # Confidence of the bounding box (Byte 1)
int8 id_confidence  # Confidence of the face ID (Byte 6)
int8 id             # Face ID (Byte 7)
uint8 is_facing     # Whether the face is facing or not
