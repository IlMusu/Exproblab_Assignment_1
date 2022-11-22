#!/usr/bin/env python

# Importing ARMOR libraries to easily interact with ARMOR
from armor_api.armor_client import ArmorClient
from armor_api.armor_utils_client import ArmorUtilsClient
from armor_manipulation_client_ext import ArmorManipulationClientExt

ONTOLOGY_PATH = '/root/Desktop/topological_map.owl'
ONTOLOGY_URI = 'http://bnc/exp-rob-lab/2022-23'

def init_ontology():
	armor_client = ArmorClient('test_client', 'assignment_ontology')
	armor_utils_client = ArmorUtilsClient(armor_client)
	armor_manipulation_client = ArmorManipulationClientExt(armor_client)
	
	armor_utils_client.load_ref_from_file(ONTOLOGY_PATH, ONTOLOGY_URI, True)
	
	armor_manipulation_client.add_objectprop_to_ind('hasDoor', 'R1', 'D1')
	armor_manipulation_client.add_objectprop_to_ind('hasDoor', 'R2', 'D2')
	armor_manipulation_client.add_objectprop_to_ind('hasDoor', 'R3', 'D3')
	armor_manipulation_client.add_objectprop_to_ind('hasDoor', 'R4', 'D4')
	
	armor_manipulation_client.add_objectprop_to_ind('hasDoor', 'C1', 'D1')
	armor_manipulation_client.add_objectprop_to_ind('hasDoor', 'C1', 'D2')
	armor_manipulation_client.add_objectprop_to_ind('hasDoor', 'C1', 'D5')
	armor_manipulation_client.add_objectprop_to_ind('hasDoor', 'C1', 'D6')
	
	armor_manipulation_client.add_objectprop_to_ind('hasDoor', 'C2', 'D3')
	armor_manipulation_client.add_objectprop_to_ind('hasDoor', 'C2', 'D4')
	armor_manipulation_client.add_objectprop_to_ind('hasDoor', 'C2', 'D5')
	armor_manipulation_client.add_objectprop_to_ind('hasDoor', 'C2', 'D7')
	
	armor_manipulation_client.add_objectprop_to_ind('hasDoor', 'E', 'D6')
	armor_manipulation_client.add_objectprop_to_ind('hasDoor', 'E', 'D7')

	armor_manipulation_client.disj_all_inds(['R1', 'R2', 'R3', 'R4', 'C1', 'C2', 'E'])
	
	armor_manipulation_client.add_objectprop_to_ind('isIn', 'Robot1', 'E')
	
	armor_manipulation_client.add_dataprop_to_ind('VisitedAt', 'R1', 'Long', '0')
	armor_manipulation_client.add_dataprop_to_ind('VisitedAt', 'R2', 'Long', '0')
	armor_manipulation_client.add_dataprop_to_ind('VisitedAt', 'R3', 'Long', '0')
	armor_manipulation_client.add_dataprop_to_ind('VisitedAt', 'R4', 'Long', '0')
	armor_manipulation_client.add_dataprop_to_ind('VisitedAt', 'C1', 'Long', '0')
	armor_manipulation_client.add_dataprop_to_ind('VisitedAt', 'C2', 'Long', '0')
	armor_manipulation_client.add_dataprop_to_ind('VisitedAt', 'E', 'Long', '0')
	
	armor_utils_client.sync_buffered_reasoner()


if __name__ == '__main__':
	init_ontology()
