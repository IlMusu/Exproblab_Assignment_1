from armor_api.armor_query_client import ArmorQueryClient

class ArmorQueryClientExt(ArmorQueryClient):

	def __init__(self, client):
		ArmorQueryClient.__init__(self, client)


	def classes_b2_ind(self, ind_name, bottom=False):
		"""
		Query the list of classes an individual belongs to.

		Args:
			ind_name (str): the name of the individual in the ontology.
			bottom (bool): if false returns all the classes, otherwise only the bottom class.

		Returns:
			list(str): the list of classes the individual belongs to.

		Raises:
			armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails.
			armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error.
		"""
		try:
			res = self._client.call('QUERY', 'CLASS', 'IND', [ind_name, str(bottom)])

		# Handling exceptions
		except rospy.ServiceException:
			raise ArmorServiceCallError(
				"Service call failed upon querying classes belonging to individual {0}".format(ind_name))
		except rospy.ROSException:
			raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")
		# Handling Armor internals errors
		if not res.success:
			raise ArmorServiceInternalError(res.error_description, res.exit_code)
		
		# Returning result 
		return res.queried_objects



