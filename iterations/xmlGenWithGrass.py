import sys

def generateXMLFile(num_prey, num_predators, num_grass, reproduce_prey_prob, reproduce_predator_prob, gain_from_food_predator):
  with open('0.xml', 'w') as f:
    f.write('<states>\n')
    f.write('<itno>0</itno>\n')
    f.write('<environment>\n')
    f.write('<REPRODUCE_PREY_PROB>' + reproduce_prey_prob + '</REPRODUCE_PREY_PROB>\n')
    f.write('<REPRODUCE_PREDATOR_PROB>' + reproduce_predator_prob + '</REPRODUCE_PREDATOR_PROB>\n')
    f.write('<GAIN_FROM_FOOD_PREDATOR>' + gain_from_food_predator + '</GAIN_FROM_FOOD_PREDATOR>\n')
    f.write('</environment>\n')
    f.write('</states>')
    print ("XML file generated successfully!")
  with open('initial_populations.txt', 'w') as f:
    f.write('' + num_prey + ' ' + num_predators + ' ' + num_grass)


if (len(sys.argv) != 7):
  print ("Correct usage: xmlGen.py num_prey num_predators num_grass reproduce_prey_prob reproduce_predator_prob gain_from_food_predator\n")
else:
  generateXMLFile(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6])  


