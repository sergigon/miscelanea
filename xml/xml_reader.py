#!/usr/bin/python
# +*+ coding: utf+8 +*+

import xml.etree.ElementTree as ET 			# To read xml file (questions and sentences)

def GetQuestion(question_type):
	"""
	@brief: Get a question from 'bingo_question.xml'
	@param string question_type: kind of question
	"""
	tree = ET.parse('bingo_questions.xml')			# Read xml file
	root = tree.getroot()										# Make a tree with the data
	#xml_path_full = "question/[@type='" + question_type + "']/etts/[@type='en']"		# Make xml path of the type
	xml_path_full = "question/[@type='" + question_type + "']/etts"		# Make xml path of the type
	xml_path_question = "question/[@type='" + question_type + "']"
	#xml_path_etts = "etts/[@type='en']"		# Make xml path of the type
	xml_path_etts = "etts"		# Make xml path of the type

	# Direct search
	etts = root.find(xml_path_full).text				# Save the data: question, grammar and answer_id
	print etts

	# By parts search
	question = root.find(xml_path_question)						# Find the question type
	etts = question.find(xml_path_etts).text				# Save the data: question, grammar and answer_id
	#etts = etts.replace('%name', 'Pepe')		# Introduce the user's name
	print etts
	grammar = question.find('grammar').text
	answer_id = question.find('answer_id').text

	print ('Question: %s\nGrammar: %s\nAnswer id: %s' % (etts.encode("utf-8"), grammar, answer_id))					
	print etts

GetQuestion('choose_card')