/*
 * Copyright (C) 2021 University of Bologna
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD license.  See the LICENSE file for details.
 *
 * Authors: Nazareno Bruschi, UniBo (<nazareno.bruschi@unibo.it>)
 *          Francesco Conti,  UniBo (<f.conti@unibo.it>)
 */


/*
 * Depends from the model. Let's keep it simple and focus on the first conv layer.
 */
#define NB_LAYER 1

#define NB_DIM_ARGS 14

/*
 * Usefuf structures to handle the network objects (i.e. layers)
 */
/* Type of layer */
typedef enum layer_type_e
{
 CONV,
 DEPTH,
 LINEAR,
 POOLING,
 RELU,
 SOFTMAX
}layer_type_t;

/* Dimensions of the layer */
typedef struct layer_dim_s
{
  int x_in;
  int y_in;
  int c_in;
  int x_out;
  int y_out;
  int c_out;
  int x_stride;
  int y_stride;
  int top_pad;
  int bot_pad;
  int lef_pad;
  int rig_pad;
  int x_ker;
  int y_ker;
} layer_dim_t;

/* network information and data pointers */
typedef struct network_layer_s
{
  int id;
  layer_type_t type;
  layer_dim_t layer_dim;
  unsigned char *input_data;
  unsigned char *output_data;
  signed char *param_data;
  unsigned char *buffer_0;
  unsigned char *buffer_1;
} network_layer_t;

int NETWORK_IDS[NB_LAYER] = {0};
int NETWORK_TYPES[NB_LAYER] = {CONV};
int NETWORK_DIMS[NB_LAYER][NB_DIM_ARGS] = {{48,48,64,48,48,64,1,1,0,0,0,0,1,1}};
