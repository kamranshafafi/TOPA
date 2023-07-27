#ifndef LOSS_CALC_H
#define LOSS_CALC_H

#include "ns3/mobility-model.h"

#include <vector>

template <typename LOSSMODEL>
bool calculateLoss(std::vector<double>& lossList,
                   ns3::Ptr<ns3::MobilityModel> ap,
                   const std::vector<ns3::Ptr<ns3::MobilityModel>>& ueList);

#endif // LOSS_CALC_H