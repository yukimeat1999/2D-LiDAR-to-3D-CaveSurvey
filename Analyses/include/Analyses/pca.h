/*
 *  pca.h
 *  Cluster
 *
 *  Created by Yuichiro Toda on 13/06/28.
 *  Copyright 2013 首都大学東京. All rights reserved.
 *
 */

#define PCADIM 3
double pca(double s_ele[][3], double cog[3], int s_ct, double ev[3]);
double pca_w(double s_ele[][3], double cog[3], int s_ct, double ev[3], double weight[]);