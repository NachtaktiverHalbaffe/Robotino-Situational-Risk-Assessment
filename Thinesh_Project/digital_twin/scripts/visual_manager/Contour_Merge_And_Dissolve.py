# https://gis.stackexchange.com/questions/63175/appending-or-merging-consecutive-line-segments-contours-as-one-single-line-of

# Import modules
import arcpy
from arcpy import env

# Define functions
def min_max(featureClass, fieldName):
    listItems = []
    rows = arcpy.SearchCursor(featureClass)
    for row in rows:
        listItems.append(row.getValue(fieldName))
    del rows
    listItems.sort()
    listMin = listItems[0]
    listMax = listItems[-1]
    return listMin, listMax

# Arguments
contourList = arcpy.GetParameterAsText(0) #input shapefiles, ElevationContours
outFolder = arcpy.GetParameterAsText(1) #output folder

# set workspace and overwrite option
arcpy.env.workspace = outFolder
arcpy.env.overwriteOutput = True
arcpy.env.outputZFlag = "Disabled"
arcpy.env.outputMFlag = "Disabled"

# local variables
dissolveFields = ["ELEV_FT"] # change field name here

# Process: Merge
arcpy.AddMessage("Merging contours...")
arcpy.Merge_management(contourList, "contour_merge.shp")

# Get Min/Max values of ELEV_FT field
ft_min, ft_max = min_max("contour_merge.shp", "ELEV_FT") # change field name here
ft_range = ft_max - ft_min

arcpy.AddMessage("ELEV_FT MIN: " + str(ft_min))
arcpy.AddMessage("ELEV_FT MAX: " + str(ft_max))

# Set progressor
arcpy.SetProgressor("step", "Breaking apart contours...", 0, ft_range, 1)
arcpy.AddMessage("Breaking apart contours...")

# Break apart and dissolve for each ELEV_FT value
mergeList = []
counter = ft_min
while counter <= ft_max:
    arcpy.SetProgressorLabel("Breaking out " + str(counter) + "ft contours...")
    arcpy.AddMessage("Attempting " + str(counter) + "ft contours...")
    counter_str = str(counter)
    counter_str = counter_str.replace("-", "neg")
    thisLyr = "elev_" + counter_str + "_lyr"
    thisDis = "elev_" + counter_str + "_dissolve.shp"
    where_clause = '"ELEV_FT" = ' + str(counter) # change field name here
    # Make Layer
    arcpy.management.MakeFeatureLayer("contour_merge.shp", thisLyr)
    # Select by attributes
    arcpy.SelectLayerByAttribute_management(thisLyr, "NEW_SELECTION", where_clause)
    # Dissolve Layer to new shape
    arcpy.Dissolve_management(thisLyr, thisDis, dissolveFields, "", "SINGLE_PART", "DISSOLVE_LINES")
    # Add to mergeList
    mergeList.append(thisDis)
    counter += 1
    arcpy.SetProgressorPosition()

arcpy.ResetProgressor()

# Merge contours
arcpy.AddMessage("Merging dissolved layers...")
arcpy.Merge_management(mergeList, "contour_dissolve.shp")

# Multipart to singlepart
arcpy.AddMessage("Converting multipart to singlepart...")
arcpy.MultipartToSinglepart_management("contour_dissolve.shp", "contour_singlepart.shp")

# Clean up
for shape in mergeList:
    arcpy.Delete_management(shape)
