#include "acq/normalEstimation.h"
#include "acq/decoratedCloud.h"
#include "acq/cloudManager.h"

#include "nanogui/formhelper.h"
#include "nanogui/screen.h"

#include "igl/readOFF.h"
#include "igl/viewer/Viewer.h"
#include <ANN/ANN.h>
#include <iostream>

class annNeighbour{
public:
    annNeighbour();
    ~annNeighbour();
    void calculateCloudNeighbours(
                                  acq::CloudT  const& cloud,
                                  acq::CloudT  const& cloud2,
                                  int k,
                                  int rows);

    
    
    
};