## Links to the synthesised dataset used for training and testing models:

1.) https://drive.google.com/file/d/1xTRD12ybKQaEmozn-2HzhUjsdc0L7I-D/view?usp=sharing contains the pair dataset from same satelite (google only)

2.) https://drive.google.com/file/d/1HzBMbTq4_i9G9rLul_eOCCx9dwUgZovq/view?usp=sharing contains the pair dataset from different satelite (google and bing)

## Naming convention of files in dataset:

(a,b)\_(c,d)\_(r)\_locationmap.png

(a,b) -> Center pixel coordinates of Reference Image wrt entire map file: x, y

(c,d) -> Relative Center pixel coordinates of Camera Image wrt Reference Image Center: x, y

(r)   -> Anticlockwise rotation angle in degrees of Camera Image wrt Reference Image north 

Coordinate system for pixel coordinates: x+ down , y+ right, origin at top left (within an image)

## Code for Dataset Generation
data_extraction_diff_map.py can be used to generate Reference Image and simulated Rotated Camera Image (contained within the Reference Image area) pairs that can be used to test Image Localization model accuracy.

The code takes input two Raster maps (can be same or different) in GeoTiff format and stores images in two corresponding folders: Reference Image and Camera Image

geotiff2png.py can be used to convert a GeoTiff raster map to an image in .png format.