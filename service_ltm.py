# rest of the imports...
import rclpy
from ltm_package.srv import LtmQuery, LtmSetModel, LtmRemember # new imports based on updated service definitions

class LongTermMemoryService:
    def __init__(self):
        self._model = ''
        self._data = []
        self._node = rclpy.create_node('long_term_memory_service')
        self._set_model_srv = self._node.create_service(LtmSetModel, 'ltm_set_model', self.set_model_callback)
        self._query_srv = self._node.create_service(LtmQuery, 'ltm_query', self.query_callback)
        self._remember_srv = self._node.create_service(LtmRemember, 'ltm_remember', self.remember_callback)

    def set_model_callback(self, request: LtmSetModel.Request, response: LtmSetModel.Response):
        if request.model_type not in [0, 1]: # changed from string comparison to integer comparison
            response.success = False
            return response
        self._model = (request.model_type, request.model_name) # changed to store both model type and name
        response.success = True
        response.model_changed = True if self._model != previous_model else False # added logic to check if the model has changed
        return response

    def query_callback(self, request: LtmQuery.Request, response: LtmQuery.Response):
        if request.query in self._data:
            response.success = True
            response.answer = "The fact is remembered." # added a sample answer for the query
        else:
            response.success = False
            response.answer = "The fact is not remembered." # added a sample answer for unsuccessful queries
        return response

    def remember_callback(self, request: LtmRemember.Request, response: LtmRemember.Response):
        if len(request.up_to_twenty_topics_of_512_chars) > 20: # added a check for the number of topics to be remembered
            response.success = False
            return response
        self._data.extend(request.up_to_twenty_topics_of_512_chars)
        response.success = True
        response.facts_remembered = len(self._data) # changed the message to be more descriptive
        return response
