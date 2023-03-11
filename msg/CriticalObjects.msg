# the label of the object
string[] labels
# the average collision risk/probability
float32[] averageRisks
# How often the object collided in relation to the total risk estimations done
float32[] relativeOccurence
# relativeOccurence * averageRisks
float32[] obstacleRisk