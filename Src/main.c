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


#include "pmsis.h"
#include "pulp_nn_kernels.h"
#include "network_desc.h"

/*
 * EXERCISE 0: Add the BSPs (Board Support Package)
 *
 * REFERENCES:
 *  - PULP-SDK
 *  - https://greenwaves-technologies.com/manuals/BUILD/PULP-OS/html/index.html
 *  - https://greenwaves-technologies.com/manuals/BUILD/PMSIS_BSP/html/md__home_yao_gap_sdk_rtos_pmsis_pmsis_bsp_docs_mainpage.html
 * 
 * DESCRIPTION:
 * Break the ice. BSPs are fundamental to communicate with off-chip devices.
 * They contains all the APIs to access and use the specific devices such as memories.
 * We will use:
 * - FILE SYSTEM
 * - FLASH
 * - RAM
 */

/*
 * Error messages from the execution
 */
#define GENERAL_ERROR    1
#define FLASH_ERROR      2
#define FILESYSTEM_ERROR 3
#define RAM_ERROR        4
#define FILE_ERROR       5
#define L2_ERROR         6
#define CLUSTER_ERROR    7
#define CHECK_ERROR      8

/*
 * Add verbosity to check the code execution
 */
//#define DEBUG
//#define PERFORMANCE

/*
 * Macro to read the data structures
 */
#define LAYER_ID(x)    NETWORK_IDS[(x)]
#define LAYER_TYPE(x)  NETWORK_TYPES[(x)]
#define LAYER_DIM(x)   *(layer_dim_t *)NETWORK_DIMS[(x)]

/*
 * Buffer in which open the files
 */
#define FLASH_BUFFER_SIZE 128
char flashBuffer[FLASH_BUFFER_SIZE];

/* File names */
char *L4_files[] = {"inputs.hex", "params0.hex", "outputs0.hex"};

/*
 * Useful variables and devices
 */
struct pi_device ram;
struct pi_device cluster;

unsigned int L3_input[NB_LAYER];
unsigned int L3_output[NB_LAYER];
unsigned int L3_param[NB_LAYER];

PI_L2 network_layer_t network_layers[NB_LAYER];

PI_L1 network_layer_t l1_layer;

/*
 * Function declarations
 */
int network_init();
int layer_init();
void cluster_init();
void kernel_init(int tileH, int tileW, int tileC);
void kernel_run();
void kernel_end(int tileH, int tileW, int tileC);
int layer_check();
void layer_run();
int main();

int network_init()
{
#if defined(DEBUG)
  printf("-> Entering Network Initialization...\n");
#endif
  /* 
   * EXERCISE 1: Flash HYPERFLASH memory
   *
   * REFERENCES:
   *  - PULP-SDK
   *  - https://greenwaves-technologies.com/manuals/BUILD/PMSIS_BSP/html/md__home_yao_gap_sdk_rtos_pmsis_pmsis_bsp_docs_mainpage.html
   * 
   * DESCRIPTION:
   * Data should be in L4 (FLASH memory). PULP-SDK allows to flash the desired data.
   * You can communicate the intentions from Makefile.
   */

  /*
   * Open the Makefile
   */

  /* 
   * The following code works only if you previously have flashed the FLASH memory, specifing it in your Makefile.
   * It configures, opens and mounts the filesystem on available FLASH memory
   */
  struct pi_device flash;
  struct pi_hyperflash_conf flash_conf;

  pi_hyperflash_conf_init(&flash_conf);
  pi_open_from_conf(&flash, &flash_conf);
   
  if (pi_flash_open(&flash))
  {
    return -FLASH_ERROR;
  }

  struct pi_device filesystem;
  struct pi_readfs_conf filesystem_conf;

  pi_readfs_conf_init(&filesystem_conf);
   
  filesystem_conf.fs.flash = &flash;

  pi_open_from_conf(&filesystem, &filesystem_conf);

  if (pi_fs_mount(&filesystem))
  {
    return -FILESYSTEM_ERROR;
  }

  /* 
   * EXERCISE 2: Navigate the filesystem and move data from L4 to L3
   *
   * REFERENCES:
   *  - PULP-SDK
   *  - https://greenwaves-technologies.com/manuals/BUILD/PULP-OS/html/index.html
   *
   * DESCRIPTION:
   * Accessing to L3 is faster than L4. Navigate L4 and move the data that you need in L3 (RAM memory).
   * L4 can be navigated as a filesystem but you need the right drivers.
   * PMSIS drivers is based on devices. pi_device is the structure used from the drivers to access
   * to particular information about the low-level device such as type of RAM, chip select, interface and so on.
   * You can find the definition of the struct in rtos/pulpos/common/pmsis_type.h
   * Every device needs to be initialized, assigning specific attributes that described the particular device.
   * Initialize with the hyperram configurations and open the global struct "ram"
   */

  struct pi_hyperram_conf ram_conf;

  /* 
   * EXERCISE 2.1
   * - Initialize ram_conf
   * - Assign it to ram device
   * - Open ram
   */

 /*
  * PUT YOUR CODE HERE
  */

  /* 
   * Now you should have the RAM ready to be used
   */

  for(int i=0; i<NB_LAYER; i++)
  {
    network_layers[i].id        = LAYER_ID(i);
    network_layers[i].type      = LAYER_TYPE(i);
    network_layers[i].layer_dim = LAYER_DIM(i);
  }

  layer_dim_t dim = network_layers[0].layer_dim;

  int tot_layer_in_dim    = dim.x_in  * dim.y_in  * dim.c_in;
  int tot_layer_out_dim   = dim.x_out * dim.y_out * dim.c_out;
  int tot_layer_param_dim = dim.x_ker * dim.y_ker * dim.c_in * dim.c_out;

  /*
   * EXERCISE 2.2
   * - Alloc space for data in L3
   *
   * REFERENCE:
   * - pulp-sdk/rtos/pmsis/pmsis_bsp/include/bsp/ram.h
   * - pulp-sdk/tests/ram
   */

  /*
   * PUT YOUR CODE HERE
   */

  pi_fs_file_t *file;
  int readDone = 0;

  /*
   * Copy the layer input
   */
  file = pi_fs_open(&filesystem, L4_files[0], 0);
  if(file == NULL)
  {
    return -FILE_ERROR;
  }

  while(readDone < tot_layer_in_dim)
  {
    int size = pi_fs_read(file, flashBuffer, FLASH_BUFFER_SIZE);

    /*
     * EXERCISE 2.3
     * - Write L3
     *
     * REFERENCE:
     * - pulp-sdk/rtos/pmsis/pmsis_bsp/include/bsp/ram.h
     * - pulp-sdk/tests/ram
     */

    /*
     * PUT YOUR CODE HERE
     */

    readDone += size;
  }

  readDone = 0;

  /*
   * Copy the layer parameter
   */
  file = pi_fs_open(&filesystem, L4_files[1], 0);
  if(file == NULL)
  {
    return -FILE_ERROR;
  }

  while(readDone < tot_layer_param_dim)
  {
    int size = pi_fs_read(file, flashBuffer, FLASH_BUFFER_SIZE);

    pi_ram_write(&ram, L3_param[0]+readDone, flashBuffer, size);

    readDone += size;
  }

  readDone = 0;

  /*
   * Copy the layer output
   */
  file = pi_fs_open(&filesystem, L4_files[2], 0);
  if(file == NULL)
  {
    return -FILE_ERROR;
  }

  while(readDone < tot_layer_out_dim)
  {
    int size = pi_fs_read(file, flashBuffer, FLASH_BUFFER_SIZE);

    pi_ram_write(&ram, L3_output[0]+readDone, flashBuffer, size);

    readDone += size;
  }

#if defined(DEBUG)
  printf("-> Exiting Network Initialization...\n");
#endif

  return 0;
}

int layer_init()
{
#if defined(DEBUG)
  printf("-> Entering Layer Initialization...\n");
#endif

  /*
   * Alloc L2 space
   */
  network_layers[0].input_data = pi_l2_malloc((unsigned int)(network_layers[0].layer_dim.x_in * network_layers[0].layer_dim.y_in * network_layers[0].layer_dim.c_in * sizeof(unsigned char)));
  if(network_layers[0].input_data == NULL)
  {
    return -L2_ERROR;
  }

  network_layers[0].param_data = pi_l2_malloc((unsigned int)(network_layers[0].layer_dim.x_ker * network_layers[0].layer_dim.y_ker * network_layers[0].layer_dim.c_in * network_layers[0].layer_dim.c_out * sizeof(signed char)));
  if(network_layers[0].param_data == NULL)
  {
    return -L2_ERROR;
  }

  network_layers[0].output_data = pi_l2_malloc((unsigned int)(network_layers[0].layer_dim.x_out * network_layers[0].layer_dim.y_out * network_layers[0].layer_dim.c_out * sizeof(unsigned char)));
  if(network_layers[0].output_data == NULL)
  {
    return -L2_ERROR;
  }

  /*
   * Copy data: inputs and parameters
   */
  pi_ram_read(&ram, (unsigned int)L3_input[0], network_layers[0].input_data, network_layers[0].layer_dim.x_in * network_layers[0].layer_dim.y_in * network_layers[0].layer_dim.c_in);

  pi_ram_read(&ram, (unsigned int)L3_param[0], network_layers[0].param_data, network_layers[0].layer_dim.x_ker * network_layers[0].layer_dim.y_ker * network_layers[0].layer_dim.c_in * network_layers[0].layer_dim.c_out);

#if defined(DEBUG)
  printf("-> Exiting Layer Initialization...\n");
#endif

  return 0;
}

/*
 * Cluster initialization
 */
void cluster_init()
{
#if defined(DEBUG)
  printf("-> Entering Cluster Initialization...\n");
#endif
  /*
   * Alloc L1 space for tiles: BE CAREFUL -> L1 is very limited
   */
  l1_layer.id   = LAYER_ID(0);
  l1_layer.type = LAYER_TYPE(0);
  
  /*
   * Local dimensions determines the tile strategy
   */
  l1_layer.layer_dim.c_in     = network_layers[0].layer_dim.c_in;
  l1_layer.layer_dim.c_out    = network_layers[0].layer_dim.c_out;
  l1_layer.layer_dim.x_ker    = network_layers[0].layer_dim.x_ker;
  l1_layer.layer_dim.y_ker    = network_layers[0].layer_dim.y_ker;
  l1_layer.layer_dim.top_pad  = network_layers[0].layer_dim.top_pad;
  l1_layer.layer_dim.bot_pad  = network_layers[0].layer_dim.bot_pad;
  l1_layer.layer_dim.lef_pad  = network_layers[0].layer_dim.lef_pad;
  l1_layer.layer_dim.rig_pad  = network_layers[0].layer_dim.rig_pad;
  l1_layer.layer_dim.x_stride = network_layers[0].layer_dim.x_stride;
  l1_layer.layer_dim.y_stride = network_layers[0].layer_dim.y_stride;

  /*
   * EXERCISE 5: Tile from L2 to L1 the data
   *
   * REFERENCES:
   *  - PULP-SDK
   *  - https://greenwaves-technologies.com/manuals/BUILD/HOME/html/index.html
   *
   * DESCRIPTION:
   * Accessing L1 memory is faster than L2. However, the size of this memory is very limited.
   * To execute the Neural Network layer in L2, you need to tile the layers' data, moving them
   * into L1 memory.
   */

  /*
   * EXERCISE 5.1
   * - Assign the tile sizes to local layer structure. Try to tile along H and W (in/out) for factor of 8, 6 and 4
   * - Verify functional and performance results using performance counters (take a screenshot)
   */

  /*
   * PUT YOUR CODE HERE
   */

  l1_layer.input_data  = pi_l1_malloc(&cluster, (unsigned int)(l1_layer.layer_dim.x_in  * l1_layer.layer_dim.y_in  * l1_layer.layer_dim.c_in * sizeof(unsigned char)));
  l1_layer.param_data  = pi_l1_malloc(&cluster, (unsigned int)(l1_layer.layer_dim.x_ker * l1_layer.layer_dim.y_ker * l1_layer.layer_dim.c_in * l1_layer.layer_dim.c_out * sizeof(signed char)));
  l1_layer.output_data = pi_l1_malloc(&cluster, (unsigned int)(l1_layer.layer_dim.x_out * l1_layer.layer_dim.y_out * l1_layer.layer_dim.c_out * sizeof(unsigned char)));

  /*
   * PULP-NN Conv kernel exploits im2col to reorder the input data: Take it into account for L1 space
   */
  if(l1_layer.type == CONV)
  {
  	l1_layer.buffer_0 = pi_l1_malloc(&cluster, (unsigned int)(2 * l1_layer.layer_dim.x_ker * l1_layer.layer_dim.y_ker * l1_layer.layer_dim.c_in * NB_CORES * sizeof(unsigned char)));
  }

#if defined(DEBUG)
  printf("-> Exiting Cluster Initialization...\n");
#endif
}

/*
 * Convolution kernel
 */
void convolution_run()
{
  /*
   * PULP-NN Convolution kernel
   */
   pulp_nn_conv(
    l1_layer.input_data,
    l1_layer.buffer_0,
    NULL,
    l1_layer.output_data,
    l1_layer.param_data,
    8,
    l1_layer.layer_dim.x_in,
    l1_layer.layer_dim.y_in,
    l1_layer.layer_dim.c_in,
    l1_layer.layer_dim.x_out,
    l1_layer.layer_dim.y_out,    
    l1_layer.layer_dim.c_out,
    l1_layer.layer_dim.x_ker,
    l1_layer.layer_dim.y_ker,
    l1_layer.layer_dim.top_pad,
    l1_layer.layer_dim.bot_pad,
    l1_layer.layer_dim.lef_pad,
    l1_layer.layer_dim.rig_pad,
    l1_layer.layer_dim.x_stride,
    l1_layer.layer_dim.y_stride);
}

/*
 * Copy data in L1
 */
void kernel_init(int tileH, int tileW, int tileC)
{
#if defined(DEBUG)
  printf("---> Entering Kernel Initialization...\n");
#endif

  pi_cl_dma_cmd_t dma_cmd;

  /*
   * EXERCISE 5.3:
   * - Call the 2D DMA copy for the input data
   */

  /*
   * PUT YOUR CODE HERE
   */

  pi_cl_dma_cmd_wait(&dma_cmd);

#if defined(DEBUG)
  printf("---> Exiting Kernel Initialization...\n");
#endif
}

/*
 * Execute the convolution kernel
 */
void kernel_run()
{
  /*
   * Fork the job over available cores
   */
  pi_cl_team_fork(NB_CORES, convolution_run, NULL);
}

/*
 * Move back the output results in L2
 */
void kernel_end(int tileH, int tileW, int tileC)
{
#if defined(DEBUG)
  printf("---> Entering Kernel Ending...\n");
#endif

  pi_cl_dma_cmd_t dma_cmd;

  /*
   * EXERCISE 5.3:
   * - Call the 2D DMA copy for the output data
   */

  /*
   * PUT YOUR CODE HERE
   */

  pi_cl_dma_cmd_wait(&dma_cmd);

#if defined(DEBUG)
  printf("---> Exiting Kernel Ending...\n");
#endif
}

/*
 * Check if the outputs are the same of the golden model
 */
int layer_check()
{
  int tot_layer_out_dim = network_layers[0].layer_dim.x_out * network_layers[0].layer_dim.y_out * network_layers[0].layer_dim.c_out;

  unsigned char *golden_model = pi_l2_malloc((unsigned int)(tot_layer_out_dim * sizeof(unsigned char)));
  if(golden_model == NULL)
  {
    return -L2_ERROR;
  }

  pi_ram_read(&ram, (unsigned int)L3_output[0], golden_model, tot_layer_out_dim);

  for(int i=0; i<tot_layer_out_dim; i++)
  {
    if(golden_model[i] != network_layers[0].output_data[i])
    {
      printf("ERROR at index %d, expected %x and got %x\n", i, golden_model[i], network_layers[0].output_data[i]);
      pi_l2_free(golden_model, (unsigned int)(tot_layer_out_dim * sizeof(unsigned char)));
      return -GENERAL_ERROR;
    }
  }

  printf("Exiting with 0 errors\n");
  pi_l2_free(golden_model, (unsigned int)(tot_layer_out_dim * sizeof(unsigned char)));

  return 0;
}

/*
 * Cluster application entry point
 */
void layer_run()
{
#if defined(DEBUG)
  printf("--> Entering Layer Running...\n");
#endif

  pi_cl_dma_cmd_t dma_cmd;

  int dma_copy_size = l1_layer.layer_dim.x_ker * l1_layer.layer_dim.y_ker * l1_layer.layer_dim.c_in * l1_layer.layer_dim.c_out;

  /*
   * Parameters copy: Let's keep it simple, avoid the tiling along channels. Move all the parameters in L1
   */
  pi_cl_dma_cmd((unsigned int)network_layers[0].param_data, (unsigned int)l1_layer.param_data, dma_copy_size, PI_CL_DMA_DIR_EXT2LOC, &dma_cmd);
  pi_cl_dma_cmd_wait(&dma_cmd);

  /*
   * Tile loop bounds
   */
  int nb_h_tile = 0;
  int nb_w_tile = 0;

  /*
   * Tile loop indexes
   */
  int h_tile = 0;
  int w_tile = 0;

  /*
   * Cluster performance counters
   */
#if defined(PERFORMANCE)
  pi_perf_conf(1 << PI_PERF_ACTIVE_CYCLES | 1 << PI_PERF_CYCLES | 1 << PI_PERF_INSTR);
  pi_perf_reset();
  pi_perf_start();
#endif

  /*
   * EXERCISE 5.2:
   * - Define the number of tiles along each dimension
   * - Implement the tile loop
   * - How does it change if you choose an odd or non-divider of H tiling factors?
   */

  /*
   * PUT YOUR CODE HERE
   */
      /*
       * Kernel initialization
       */
      kernel_init(h_tile, w_tile, 0);

      /*
       * Executing the main kernel
       */
      kernel_run();

      /*
       * Kernel ending
       */
      kernel_end(h_tile, w_tile, 0);

#if defined(PERFORMANCE)
  pi_perf_stop();
  uint32_t instr_cnt      = pi_perf_read(PI_PERF_INSTR);
  uint32_t cycles_cnt     = pi_perf_read(PI_PERF_CYCLES);
  uint32_t act_cycles_cnt = pi_perf_read(PI_PERF_ACTIVE_CYCLES);
  printf("[0]: instructions = %d, tot_cycles = %d, active_cycles = %d \n", instr_cnt, cycles_cnt, act_cycles_cnt);
#endif

#if defined(DEBUG)
  printf("--> Exiting Layer Running...\n");
#endif
}

/*
 * FC application entry point
 */
int main()
{
  printf("Entering Main...\n");
  /*
   * Initialize the network and copy data in L3
   */
  int err = network_init();
  if(err)
  {
    return err;
  }

  /*
   * Copy data of the single layer in L2 (if it is possible)
   */
  err = layer_init();
  if(err)
  {
    return err;
  }

  /*
   * Now you should have all the parameters in L2 to compute the output of the first layer
   */

  /* 
   * EXERCISE 3: Use the force of the PULP Cluster
   *
   * REFERENCES:
   *  - PULP-SDK
   *  - https://greenwaves-technologies.com/manuals/BUILD/PMSIS_API/html/group__clusterDriver.html
   *  - https://greenwaves-technologies.com/manuals/BUILD/PMSIS_API/html/group__FcClusterSync.html
   *
   * DESCRIPTION:
   * PULP Cluster has 8 identical cores that can share and compute heavy computational workload in parallel.
   * PULP Cluster is a PMSIS device. Open it and send the task to initialize it.
   */

  struct pi_cluster_conf cluster_conf;

  /* 
   * EXERCISE 3.0
   * - Configure and open cluster. cluster is a global struct
   *
   * REFERENCES:
   * - pulp-sdk/rtos/pmsis/pmsis_api/include/pmsis/cluster/cluster_sync
   * - pulp-sdk/rtos/pulpos/common/kernel/device.c
   * - pulp-sdk/tests/cluster/call
   */

  /*
   * PUT YOUR CODE HERE
   */

  struct pi_cluster_task cluster_task;

  /* 
   * EXERCISE 3.1
   * - Configure the init task and send it to cluster
   *
   * REFERENCES:
   * - pulp-sdk/rtos/pmsis/pmsis_api/include/pmsis/cluster/cluster_sync
   * - pulp-sdk/tests/cluster
   */
  
  /*
   * PUT YOUR CODE HERE
   */

  /* 
   * EXERCISE 4: Profile the execution and send the main task to the cluster
   *
   * REFERENCES:
   *  - PULP-SDK
   *  - https://greenwaves-technologies.com/manuals/BUILD/PMSIS_API/html/group__Perf.html
   *
   * DESCRIPTION: 
   * To profile the execution of a given code, activate the performance counters to count events of 
   * different kinds (cycles, instructions, load stalls, ...).
   * In GVSoC, all the performance counters are available at the same time.
   */

  /*
   * EXERCISE 4.1
   * - Configure, reset and start the performance counters for cycles and instructions
   *
   * REFERENCES:
   * - pulp-sdk/rtos/pmsis/pmsis_api/include/pmsis/chips/default.h
   * - pulp-sdk/rtos/pmsis/pmsis_api/include/pmsis/drivers/perf.h
   * - pulp-sdk/tests/matmult
   */

  /*
   * PUT YOUR CODE HERE
   */

  /*
   * EXERCISE 4.2
   * - Configure the main task and send it to cluster
   *
   * REFERENCES:
   * - pulp-sdk/rtos/pmsis/pmsis_api/include/pmsis/cluster/cluster_sync
   * - pulp-sdk/tests/cluster
   */

  /*
   * PUT YOUR CODE HERE
   */

  /*
   * EXERCISE 4.3
   * - Stop and read the performance counters
   *
   * REFERENCES:
   * - pulp-sdk/rtos/pmsis/pmsis_api/include/pmsis/chips/default.h
   * - pulp-sdk/rtos/pmsis/pmsis_api/include/pmsis/drivers/perf.h
   * - pulp-sdk/tests/matmult
   */

  /*
   * PUT YOUR CODE HERE
   */

  /*
   * Release the resources
   */
  pi_l1_free(&cluster, l1_layer.input_data , (unsigned int)(l1_layer.layer_dim.x_in  * l1_layer.layer_dim.y_in  * l1_layer.layer_dim.c_in * sizeof(unsigned char)));
  pi_l1_free(&cluster, l1_layer.param_data , (unsigned int)(l1_layer.layer_dim.x_ker * l1_layer.layer_dim.y_ker * l1_layer.layer_dim.c_in * l1_layer.layer_dim.c_out * sizeof(signed char)));
  pi_l1_free(&cluster, l1_layer.output_data, (unsigned int)(l1_layer.layer_dim.x_out * l1_layer.layer_dim.y_out * l1_layer.layer_dim.c_out * sizeof(unsigned char)));

  pi_cluster_close(&cluster);

  pi_l2_free(network_layers[0].input_data, (unsigned int)(network_layers[0].layer_dim.x_in  * network_layers[0].layer_dim.y_in  * network_layers[0].layer_dim.c_in * sizeof(unsigned char)));
  pi_l2_free(network_layers[0].param_data, (unsigned int)(network_layers[0].layer_dim.x_ker * network_layers[0].layer_dim.y_ker * network_layers[0].layer_dim.c_in * network_layers[0].layer_dim.c_out * sizeof(signed char)));

  /*
   * Check the output results
   */
  err = layer_check();
  if(err)
  {
  	return err;
  }

  pi_l2_free(network_layers[0].output_data, (unsigned int)(network_layers[0].layer_dim.x_out * network_layers[0].layer_dim.y_out * network_layers[0].layer_dim.c_out * sizeof(unsigned char)));

  pi_ram_free(&ram, (unsigned int)L3_input[0],  (unsigned int)(network_layers[0].layer_dim.x_in  * network_layers[0].layer_dim.y_in  * network_layers[0].layer_dim.c_in * sizeof(unsigned char)));
  pi_ram_free(&ram, (unsigned int)L3_output[0], (unsigned int)(network_layers[0].layer_dim.x_out * network_layers[0].layer_dim.y_out * network_layers[0].layer_dim.c_out * sizeof(unsigned char)));
  pi_ram_free(&ram, (unsigned int)L3_param[0],  (unsigned int)(network_layers[0].layer_dim.x_ker * network_layers[0].layer_dim.y_ker * network_layers[0].layer_dim.c_in * network_layers[0].layer_dim.c_out * sizeof(signed char)));

  pi_ram_close(&ram);

  printf("Exiting Main...\n");

  return 0;
}
