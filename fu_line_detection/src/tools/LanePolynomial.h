/*
 * LanePolynomial.h
 *
 *  Created on: 16.12.2014
 *      Author: conrad
 */

#ifndef SRC_REPRESENTATIONS_COGNITION_VISION_LANEPOLYNOMIAL_H_
#define SRC_REPRESENTATIONS_COGNITION_VISION_LANEPOLYNOMIAL_H_

#include "NewtonPolynomial.h"

class LanePolynomial {
public:
    LanePolynomial();
    virtual ~LanePolynomial();

    const NewtonPolynomial& getLanePoly() const;
    const std::vector<FuPoint<int>>& getPoints() const;
    bool hasDetected() const;

    void setLanePoly(NewtonPolynomial);
    void setPoints(std::vector<FuPoint<int>>);

    void setNotDetected();
    void setDetected();

private:
    NewtonPolynomial lanePoly;
    std::vector<FuPoint<int>> lanePoints;

    bool lanesDetected;
};

#endif /* SRC_REPRESENTATIONS_COGNITION_VISION_LANEPOLYNOMIAL_H_ */
