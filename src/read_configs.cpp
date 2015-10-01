#include <string>
#include <string.h>
#include <vector>
#include <libconfig.h>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <liquid/liquid.h>
#include "read_configs.hpp"
#include "node_parameters.hpp"

int read_scenario_master_file(char scenario_list[30][60], unsigned int scenario_reps[60])
{
    config_t cfg; // Returns all parameters in this structure
    char current_scenario[30];
    const char *tmpS;
    int num_scenarios = 1;
    int tmpI; // Stores the value of Integer Parameters from Config file
    
    config_init(&cfg);
    
    // Read the file. If there is an error, report it and exit.
    if (!config_read_file(&cfg, "master_scenario_file.cfg")){
        printf("Error reading master scenario file on line %i\n", config_error_line(&cfg));
        exit(1);
    }
    
    // Read the parameter group
    if (config_lookup_int(&cfg, "NumberofScenarios", &tmpI))
        num_scenarios = (int)tmpI;
        
    for (int i=0; i< num_scenarios; i++){
        sprintf(current_scenario, "scenario_%d", i+1);
        if (config_lookup_string(&cfg, current_scenario, &tmpS))
            strcpy(&scenario_list[i][0], tmpS);
    }
    for (int i=0; i< num_scenarios; i++){
        sprintf(current_scenario, "reps_scenario_%d", i+1);
        if (config_lookup_int(&cfg, current_scenario, &tmpI))
            scenario_reps[i] = tmpI;
    }
    config_destroy(&cfg);
    return num_scenarios;
} // End readScMasterFile()

struct scenario_parameters read_scenario_parameters(char * scenario_file)
{
    // configuration variable
    config_t cfg;
    config_init(&cfg);

    // string pointing to scenario file
    char scenario[100];
    strcpy(scenario, "scenarios/");
    strcat(scenario, scenario_file);

    // Read the file. If there is an error, report it and exit.
    if (!config_read_file(&cfg, scenario)){
        printf("Error reading %s on line %i\n", scenario, config_error_line(&cfg));
        printf("%s\n", config_error_text(&cfg));
        config_destroy(&cfg);
        exit(1);
    }

    // Read scenario parameters
    struct scenario_parameters sp;
    int tmpI;
    double tmpD;

    config_lookup_int(&cfg, "num_nodes", &tmpI);
    sp.num_nodes = tmpI;
    config_lookup_float(&cfg, "run_time", &tmpD);
    sp.run_time = (float) tmpD;
    config_destroy(&cfg);

    return sp;
} // End readScConfigFile()

struct node_parameters read_node_parameters(int node, char *scenario_file){
    // string pointing to scenario file
    char scenario[100];
    strcpy(scenario, "scenarios/");
    strcat(scenario, scenario_file);

    config_t cfg;
    config_init(&cfg);

    // Read the file. If there is an error, report it and exit.
    if (!config_read_file(&cfg, scenario))
    {
        printf("Error reading config file on line %i\n", config_error_line(&cfg));
        config_destroy(&cfg);
        exit(1);
    }
    
    int tmpI;
    double tmpD;
    const char *tmpS;
    
    // scenario info struct for node
    struct node_parameters np = {};
    char nodestr[100];
    std::string node_num;
    std::stringstream out;
    out << node;
    node_num = out.str();
    

    // lookup specific node
    strcpy(nodestr, "node");
    strcat(nodestr, node_num.c_str());
    config_setting_t *node_config = config_lookup(&cfg, nodestr);

    // read CORNET IP address for the node
    if (config_setting_lookup_string(node_config, "TARGET_IP", &tmpS))
        strcpy(np.TARGET_IP, tmpS);
	else
		strcpy(np.TARGET_IP, "10.0.0.3");

    // read CORNET IP address for the node
    if (config_setting_lookup_string(node_config, "CORNET_IP", &tmpS))
        strcpy(np.CORNET_IP, tmpS);
	else
		strcpy(np.CORNET_IP, "192.168.1.12");

    // read type of node
    if (config_setting_lookup_string(node_config, "type", &tmpS))
    {
        //printf("\nNode type: %s\n", tmpS);
        if (strcmp(tmpS, "CR") == 0) 
        {
            np.type = CR;
            np.cr_type = ecr;
            //If node is a CR, lookup whether is uses the ECR or python
            if(config_setting_lookup_string(node_config, "cr_type", &tmpS))
            {
                if(strcmp(tmpS, "python") == 0)
                {
                    np.cr_type = python;
                    //python radios are specified by the "python_file" field in the scenario file
                    if(config_setting_lookup_string(node_config, "python_file", &tmpS))
                    {
                        strcpy(np.python_file, tmpS);
                        //check for optional command line arguments and add them to
                        //np.arguments if found
                        const config_setting_t *arguments;
                        char path[100];
                        strcpy(path, nodestr);
                        strcat(path, ".arguments");
                        arguments = config_lookup(&cfg, path);
                        np.num_arguments = 0;
                        if(arguments != NULL)
                        {
                            np.num_arguments = config_setting_length(arguments);
                            for(int i = 0; i < np.num_arguments; i++)
                            {
                                tmpS = config_setting_get_string_elem(arguments, i);
                                strcpy(np.arguments[i], tmpS);
                            }
                        }
                        else
                            printf("arguments not found\n");
                    }
                    else
                    {
                        printf("A python radio requires a python file.\n");
                    }
                }
                //Possibly add more types later, but for now if not python, then radio must be ECR-based
                // and the engine to use is specified by the "CE" field
                else
                {
                    if (config_setting_lookup_string(node_config, "CE", &tmpS))
                        strcpy(np.CE, tmpS);
					else{
						printf("Configuration of a node did not specify a cognitive engine");
						exit(1);
					}
                }
            }
        }
        else if (strcmp(tmpS, "interferer") == 0) np.type = interferer;
    }

    // read all possible node settings
    if (config_setting_lookup_string(node_config, "CRTS_IP", &tmpS))
        strcpy(np.CRTS_IP, tmpS);
	else
		strcpy(np.CRTS_IP, "10.0.0.2");

    if (config_setting_lookup_int(node_config, "print_metrics", &tmpI))
        np.print_metrics = (int)tmpI;
    
    if (config_setting_lookup_int(node_config, "log_rx_metrics", &tmpI))
        np.log_rx_metrics = (int)tmpI;

    if (config_setting_lookup_int(node_config, "log_tx_parameters", &tmpI))
        np.log_tx_parameters = (int)tmpI;
    
    if (config_setting_lookup_int(node_config, "log_CRTS_rx_data", &tmpI))
        np.log_CRTS_rx_data = (int)tmpI;
    
    if (config_setting_lookup_string(node_config, "rx_log_file", &tmpS))
        strcpy(np.rx_log_file, tmpS);

    if (config_setting_lookup_string(node_config, "tx_log_file", &tmpS))
        strcpy(np.tx_log_file, tmpS);

    if (config_setting_lookup_string(node_config, "CRTS_rx_log_file", &tmpS))
        strcpy(np.CRTS_rx_log_file, tmpS);

    if (config_setting_lookup_int(node_config, "generate_octave_logs", &tmpI))
        np.generate_octave_logs = (int)tmpI;
    
    if (config_setting_lookup_int(node_config, "generate_python_logs", &tmpI))
        np.generate_python_logs = (int)tmpI;
    
    if (config_setting_lookup_float(node_config, "ce_timeout_ms", &tmpD))
        np.ce_timeout_ms = tmpD;

    if (config_setting_lookup_string(node_config, "duplex", &tmpS)){
        if(!strcmp(tmpS, "FDD")) np.duplex = FDD;
        else if(!strcmp(tmpS, "TDD")) np.duplex = TDD;
        else if(!strcmp(tmpS, "HD")) np.duplex = HD;
		else np.duplex = FDD;
    }

    if (config_setting_lookup_float(node_config, "tx_freq", &tmpD))
        np.tx_freq = tmpD;

    if (config_setting_lookup_float(node_config, "rx_freq", &tmpD))
        np.rx_freq = tmpD;

    if (config_setting_lookup_float(node_config, "tx_rate", &tmpD))
        np.tx_rate = tmpD;
	else
		np.rx_rate = 1e6;

    if (config_setting_lookup_float(node_config, "rx_rate", &tmpD))
        np.rx_rate = tmpD;
	else
		np.rx_rate = 1e6;

    if (config_setting_lookup_float(node_config, "tx_gain_soft", &tmpD))
        np.tx_gain_soft = tmpD;
	else
		np.tx_gain_soft = -12.0;

    if (config_setting_lookup_float(node_config, "tx_gain", &tmpD))
        np.tx_gain = tmpD;
	else
		np.tx_gain = 20.0;

    if (config_setting_lookup_float(node_config, "rx_gain", &tmpD))
        np.rx_gain = tmpD;
	else
		np.rx_gain = 20.0;

    // default rx modulation is BPSK
    np.tx_modulation = LIQUID_MODEM_BPSK;
    if (config_setting_lookup_string(node_config, "rx_modulation", &tmpS)){

        // Iterate through every liquid modulation scheme 
        // and if the string matches, then assign that scheme.
        // See liquid soruce: src/modem/src/modem_utilities.c
        // for definition of modulation_types
        for (int k = 0; k<LIQUID_MODEM_NUM_SCHEMES; k++){
            if(!strcmp(tmpS, modulation_types[k].name))
                np.rx_modulation = modulation_types[k].scheme;
        }
    }

    // default rx CRC32
    np.rx_crc = LIQUID_CRC_32;
    if (config_setting_lookup_string(node_config, "rx_crc", &tmpS)){

        // Iterate through every liquid CRC
        // and if the string matches, then assign that CRC.
        // See liquid soruce: src/fec/src/crc.c
        // for definition of crc_scheme_str
        for (int k = 0; k<LIQUID_CRC_NUM_SCHEMES; k++){
            if(!strcmp(tmpS, crc_scheme_str[k][0]))
                np.rx_crc = k;
        }
    }

    // default rx FEC0 is Hamming 12/8
    np.rx_fec0 = LIQUID_FEC_HAMMING128;
    if (config_setting_lookup_string(node_config, "rx_fec0", &tmpS)){

        // Iterate through every liquid FEC
        // and if the string matches, then assign that FEC.
        // See liquid soruce: src/fec/src/fec.c
        // for definition of fec_scheme_str
        for (int k = 0; k<LIQUID_FEC_NUM_SCHEMES; k++){
            if(!strcmp(tmpS, fec_scheme_str[k][0]))
                np.rx_fec0 = k;
        }
    }

    // default rx FEC1 is none
    np.rx_fec1 = LIQUID_FEC_NONE;
    if (config_setting_lookup_string(node_config, "rx_fec1", &tmpS)){

        // Iterate through every liquid FEC
        // and if the string matches, then assign that FEC.
        // See liquid soruce: src/fec/src/fec.c
        // for definition of fec_scheme_str
        for (int k = 0; k<LIQUID_FEC_NUM_SCHEMES; k++){
            if(!strcmp(tmpS, fec_scheme_str[k][0]))
                np.rx_fec1 = k;
        }
    }
    
	// default tx modulation is BPSK
    np.tx_modulation = LIQUID_MODEM_BPSK;
    if (config_setting_lookup_string(node_config, "tx_modulation", &tmpS)){

        // Iterate through every liquid modulation scheme 
        // and if the string matches, then assign that scheme.
        // See liquid soruce: src/modem/src/modem_utilities.c
        // for definition of modulation_types
        for (int k = 0; k<LIQUID_MODEM_NUM_SCHEMES; k++){
            if(!strcmp(tmpS, modulation_types[k].name))
                np.tx_modulation = modulation_types[k].scheme;
        }
	}

	
    // default tx CRC32
    np.tx_crc = LIQUID_CRC_32;
    if (config_setting_lookup_string(node_config, "tx_crc", &tmpS)){

        // Iterate through every liquid CRC
        // and if the string matches, then assign that CRC.
        // See liquid soruce: src/fec/src/crc.c
        // for definition of crc_scheme_str
        for (int k = 0; k<LIQUID_CRC_NUM_SCHEMES; k++){
            if(!strcmp(tmpS, crc_scheme_str[k][0]))
                np.tx_crc = k;
        }
    }

    // default tx FEC0 is Hamming 12/8
    np.tx_fec0 = LIQUID_FEC_HAMMING128;
    if (config_setting_lookup_string(node_config, "tx_fec0", &tmpS)){

        // Iterate through every liquid FEC
        // and if the string matches, then assign that FEC.
        // See liquid soruce: src/fec/src/fec.c
        // for definition of fec_scheme_str
        for (int k = 0; k<LIQUID_FEC_NUM_SCHEMES; k++){
            if(!strcmp(tmpS, fec_scheme_str[k][0]))
                np.tx_fec0 = k;
        }
    }

    // default rx FEC1 is none
    np.tx_fec1 = LIQUID_FEC_NONE;
    if (config_setting_lookup_string(node_config, "tx_fec1", &tmpS)){

        // Iterate through every liquid FEC
        // and if the string matches, then assign that FEC.
        // See liquid soruce: src/fec/src/fec.c
        // for definition of fec_scheme_str
        for (int k = 0; k<LIQUID_FEC_NUM_SCHEMES; k++){
            if(!strcmp(tmpS, fec_scheme_str[k][0]))
                np.tx_fec1 = k;
        }
    }
    if (config_setting_lookup_float(node_config, "tx_delay_us", &tmpD))
        np.tx_delay_us = tmpD;
	else
		np.tx_delay_us = 1e4;

    if (config_setting_lookup_string(node_config, "interference_type", &tmpS)){
        if(!strcmp(tmpS, "CW"))
            np.interference_type = CW;
        if(!strcmp(tmpS, "AWGN")) 
            np.interference_type = AWGN;
        if(!strcmp(tmpS, "GMSK"))
            np.interference_type = GMSK;
        if(!strcmp(tmpS, "RRC")) 
            np.interference_type = RRC;
        if(!strcmp(tmpS, "OFDM")) 
            np.interference_type = OFDM;
    }
    
    if (config_setting_lookup_float(node_config, "period", &tmpD))
        np.period = tmpD;

    if (config_setting_lookup_float(node_config, "duty_cycle", &tmpD))
        np.duty_cycle = tmpD;

    // ======================================================
    // process frequency hopping parameters
    // ======================================================
    if (config_setting_lookup_string(node_config, 
                                     "tx_freq_hop_type", 
                                     &tmpS))
          {
      if(!strcmp(tmpS, "NONE"))
        np.tx_freq_hop_type = NONE;
      if(!strcmp(tmpS, "ALTERNATING"))
        np.tx_freq_hop_type = ALTERNATING;
      if(!strcmp(tmpS, "SWEEP")) 
        np.tx_freq_hop_type = SWEEP;
      if(!strcmp(tmpS, "RANDOM"))
        np.tx_freq_hop_type = RANDOM;
      }
    
    if (config_setting_lookup_float(node_config, 
                                    "tx_freq_hop_min", 
                                    &tmpD))
        np.tx_freq_hop_min = tmpD;

    if (config_setting_lookup_float(node_config, 
                                    "tx_freq_hop_max", 
                                    &tmpD))
        np.tx_freq_hop_max = tmpD;

    if (config_setting_lookup_float(node_config, 
                                    "tx_freq_hop_dwell_time", 
                                    &tmpD))
        np.tx_freq_hop_dwell_time = tmpD;

    if (config_setting_lookup_float(node_config, 
                                    "tx_freq_hop_increment", 
                                    &tmpD))
        np.tx_freq_hop_increment = tmpD;


        // ======================================================
        // process GMSK interference parameters
        // ======================================================
    if (config_setting_lookup_float(node_config, 
                                    "gmsk_header_length", 
                                    &tmpD))
        np.gmsk_header_length = tmpD;

    if (config_setting_lookup_float(node_config, 
                                    "gmsk_payload_length", 
                                    &tmpD))
        np.gmsk_payload_length = tmpD;

    if (config_setting_lookup_float(node_config, 
                                    "gmsk_bandwidth", 
                                    &tmpD))
        np.gmsk_bandwidth = tmpD;



    return np;
}

void print_node_parameters(struct node_parameters * np)
  {
  printf("\n");
  printf("--------------------------------------------------------------\n");
  printf("-                    node parameters                         -\n");
  printf("--------------------------------------------------------------\n");
  printf("General Settings:\n");
  char node_type[15] = "UNKNOWN";
  if(np->type == CR) 
    {
    strcpy(node_type, "CR");
    }
  else if(np->type == interferer) 
    {
    strcpy(node_type, "Interferer");
    }

  printf("    Node type:                         %-s\n", node_type);
  if(np->type == CR)
  {
      char cr_type[15] = "ecr";
      if(np->cr_type == python)
          strcpy(cr_type, "python");
      printf("    CR type:                           %-s\n", cr_type);
  }
  printf("    CORNET IP:                         %-s\n", np->CORNET_IP);
  //
  if(np->type != interferer)
    {
    printf("\nVirtual Network Interface Settings:\n");
    printf("    CRTS IP:                           %-s\n", np->CRTS_IP);
    printf("    Target IP:                         %-s\n", np->TARGET_IP);
    //
	printf("\nCognitive Engine Settings:\n");
	printf("    Cognitive Engine:                  %-s\n", np->CE);
    printf("    CE timeout:                        %-.2f\n", np->ce_timeout_ms);
    }
  //
  printf("\nLog/Report Settings:\n");
  if(np->type != interferer)
    printf("    Rx log file:                       %-s\n", np->rx_log_file);
  printf("    Tx log file:                       %-s\n", np->tx_log_file);
  if(np->type != interferer)
    printf("    CRTS Rx log file:                  %-s\n", np->CRTS_rx_log_file);
  printf("    Generate octave logs:              %i\n", np->generate_octave_logs);
  printf("    Generate python logs:              %i\n", np->generate_python_logs);
  //
  printf("\nInitial USRP Settings:\n");
  if(np->type != interferer)
    {
    printf("    Receive frequency:                 %-.2e\n", np->rx_freq);
    printf("    Receive rate:                      %-.2e\n", np->rx_rate);
    printf("    Receive gain:                      %-.2f\n", np->rx_gain);
  	}
  printf("    Transmit frequency:                %-.2e\n", np->tx_freq);
  printf("    Transmit rate:                     %-.2e\n", np->tx_rate);
  printf("    Transmit gain:                     %-.2f\n", np->tx_gain);
  //
  if(np->type != interferer && np->cr_type == ecr)
    {
    printf("\nInitial Liquid OFDM Settings\n");
	char duplex[4] = "FDD";
    switch(np->duplex)
      {
      case (FDD): strcpy(duplex, "FDD"); break;
      case (TDD): strcpy(duplex, "TDD"); break;
      case (HD): strcpy(duplex, "HD"); break;
      }
    printf("    Duplex scheme:                     %-s\n", duplex);
    printf("    Receive modulation:                %s\n", modulation_types[np->rx_modulation].name);
    printf("    Receive CRC:                       %s\n", crc_scheme_str[np->rx_crc][0]);
    printf("    Receive FEC0:                      %s\n", fec_scheme_str[np->rx_fec0][0]); 
	printf("    Receive FEC1:                      %s\n", fec_scheme_str[np->rx_fec1][0]); 
	printf("    Transmit soft gain:                %-.2f\n", np->tx_gain_soft);
    printf("    Transmit modulation:               %s\n", modulation_types[np->tx_modulation].name);
    printf("    Transmit CRC:                      %s\n", crc_scheme_str[np->tx_crc][0]);
    printf("    Transmit FEC0:                     %s\n", fec_scheme_str[np->tx_fec0][0]); 
	printf("    Transmit FEC1:                     %s\n", fec_scheme_str[np->tx_fec1][0]); 
	}

  if(np->type == interferer)
    {
    printf("\nInitial Interference Settings:\n");
	char interference_type[5] = "NONE";
    char tx_freq_hop_type[6] = "NONE"; 
    switch(np->interference_type)
      {
      case (CW): strcpy(interference_type, "CW"); break;
      case (AWGN): strcpy(interference_type, "AWGN"); break;
      case (GMSK): strcpy(interference_type, "GMSK"); break;
      case (RRC): strcpy(interference_type, "RRC"); break;
      case (OFDM): strcpy(interference_type, "OFDM"); break;
      }
    switch(np->tx_freq_hop_type)
      {
      case (NONE): strcpy(tx_freq_hop_type, "NONE"); break;
      case (ALTERNATING): strcpy(tx_freq_hop_type, "ALTERNATING"); break;
      case (SWEEP): strcpy(tx_freq_hop_type, "SWEEP"); break;
      case (RANDOM): strcpy(tx_freq_hop_type, "RANDOM"); break;
      }
    printf("    Interference type:                 %-s\n", interference_type);
    printf("    Interference period:       %-.2f\n", np->period);
    printf("    Interference duty cycle:           %-.2f\n", np->duty_cycle);
    printf("\n"); 
    printf("    tx freq hop type:                  %-s\n", tx_freq_hop_type);
    printf("    tx freq hop min:                   %-.2e\n", np->tx_freq_hop_min);
    printf("    tx freq hop max:                   %-.2e\n", np->tx_freq_hop_max);
    printf("    tx freq hop dwell time:            %-.2f\n", np->tx_freq_hop_dwell_time);
    printf("    tx freq hop increment:             %-.2e\n", np->tx_freq_hop_increment);

    printf("\n"); 
    printf("    GMSK header length:                %-.2d\n", np->gmsk_header_length);
    printf("    GMSK payload length:               %-.2d\n", np->gmsk_payload_length);
    printf("    GMSK bandwidth:                    %-.2e\n", np->gmsk_bandwidth);
    }
  printf("--------------------------------------------------------------\n");
  }










