import sys

def generateXMLFile(num_predators, num_prey, reproduce_predator_prob, reproduce_prey_prob, gain_from_food_predator):
  with open('0.xml', 'w') as f:
    f.write('<states>\n')
    f.write('<itno>0</itno>\n')
    f.write('<environment>\n')
    f.write('<REPRODUCE_PREY_PROB>' + reproduce_prey_prob + '</REPRODUCE_PREY_PROB>\n')
    f.write('<REPRODUCE_PREDATOR_PROB>' + reproduce_predator_prob + '</REPRODUCE_PREDATOR_PROB>\n')
    f.write('<GAIN_FROM_FOOD_PREDATOR>' + gain_from_food_predator + '</GAIN_FROM_FOOD_PREDATOR>\n')
    f.write('<NUM_PREDATORS>' + num_predators + '</NUM_PREDATORS>\n')
    f.write('<NUM_PREY>' + num_prey + '</NUM_PREY>\n')
    f.write('</environment>\n')
    f.write('</states>')
    print ("XML file generated successfully!")


if (len(sys.argv) != 6):
  print ("Correct usage: xmlGen.py num_predators num_prey reproduce_predator_prob reproduce_prey_prob gain_from_food_predator\n")
else:
  generateXMLFile(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5])  


