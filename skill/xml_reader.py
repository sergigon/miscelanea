#!/usr/bin/python
# +*+ coding: utf+8 +*+

import xml.etree.ElementTree as ET # To read xml file (questions and sentences)

class XMLReader():
	"""
	XML Reader class
	"""

	def __init__(self, datapath):
		"""
		Init method.

		@param datapath: Data path from where to work with data. Usually, the package path.
		"""

		# Gets paths
		self._data_path = datapath # Data path
		self._expressions_path = self._data_path + 'etts_expressions/' # Etts expressions path
		#self._cache_path = self._data_path + 'cache/' # Data path

	def GetQuestion(self, question_type, language, name=''):
		"""
		@brief: Get a question from 'skill_question.xml'
		@param string question_type: kind of question
		"""
		tree = ET.parse(self._expressions_path + 'skill_questions.xml') # Read xml file
		root = tree.getroot() # Make a tree with the data
		question_info_path = "question/[@type='" + question_type + "']/language/[@type='" + language + "']"

		# Save data
		question = root.find(question_info_path) # Find the question type
		etts = question.find('etts').text
		etts = etts.encode("utf-8")
		etts = etts.replace('%name', name) # Introduce the user's name
		grammar = question.find('grammar').text
		answer_id = question.find('answer_id').text

		print ('Question: %s\nGrammar: %s\nAnswer id: %s' % (etts, grammar, answer_id))

		return etts, grammar, answer_id

	def GetExpression(self, question_type, language, name=''):
		"""
		@brief: Get a question from 'skill_question.xml'
		@param string question_type: kind of question
		"""
		tree = ET.parse(self._expressions_path + 'skill_expressions.xml') # Read xml file
		root = tree.getroot() # Make a tree with the data
		question_info_path = "expression/[@type='" + question_type + "']/language/[@type='" + language + "']"

		# Save data
		question = root.find(question_info_path) # Find the question type
		etts = question.find('etts').text
		etts = etts.encode("utf-8")
		etts = etts.replace('%name', name) # Introduce the user's name
		answer_id = question.find('answer_id').text

		print ('Question: %s\nAnswer id: %s' % (etts, answer_id))

		return etts, answer_id
