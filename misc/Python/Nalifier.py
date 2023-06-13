import sys
import math

class ValueReporter:
  def __init__(self, AIKR_Limit=1000):
    self.values = []
    self.AIKR_Limit = AIKR_Limit
  def reportValue(self, x, Sensation_Reliance = 0.9, RangeUpdate=True, Uniform_Weight=False, Print=False):
    if RangeUpdate:
        self.values.append(x)
        if self.AIKR_Limit != None:
            self.values = self.values[-self.AIKR_Limit:]
    wplus = 0
    wminus = 0
    for y in self.values:
      weight = 1.0 if Uniform_Weight else abs(x - y)
      if y < x:
        wplus += weight
      elif y > x:
        wminus += weight
    w = wplus+wminus
    frequency = wplus/(wplus+wminus) if w > 0 else 0.5
    confidence = w/(w+1.0) if w > 0 else 0.0
    Sensed_Value_Truth = (frequency, confidence * Sensation_Reliance)
    if Print:
        print("<{S} --> [P]>. :|:", str(Sensed_Value_Truth).replace("(","{").replace(")","}"), "from value", x)
    return Sensed_Value_Truth

def Truth_c2w(c):
    return c / (1 - c);

def Truth_w2c(w):
    return w/(w + 1) if w > 0 else 0

def Truth_w2f(w_plus, w_minus):
    return w_plus / (w_plus + w_minus) if w_plus + w_minus > 0 else 0.5

def TruthValue(w_plus, w_minus):
    return (Truth_w2f(w_plus, w_minus), Truth_w2c(w_plus + w_minus))
    
def Truth_Abduction(v1, v2):
    ((f1,c1), (f2,c2)) = (v1, v2)
    return (f2, Truth_w2c(f1 * c1 * c2))

def Truth_Induction(v1, v2):
    return Truth_Abduction(v2, v1)

def Truth_Comparison(v1, v2):
    ((f1,c1), (f2, c2)) = (v1, v2)   
    f0 = 1.0 - (1.0 - f1) * (1.0 - f2)
    return (0.0 if f0 == 0.0 else ((f1*f2) / f0), Truth_w2c(f0 * c1 * c2))

def Truth_FrequencyComparison(v1, v2):
    ((f1,c1), (f2,c2)) = (v1, v2)
    return (1.0-abs(f1 - f2), c1*c2)

def Truth_Negation(v1):
    (f, c) = v1
    return (1.0-f, c)

def Truth_Revision(v1, v2):
    MAX_CONFIDENCE = 0.99
    ((f1,c1), (f2,c2)) = (v1, v2)
    w1 = Truth_c2w(c1)
    w2 = Truth_c2w(c2)
    w = w1 + w2
    if w == 0.0:
        return (0.5, 0.0)
    return ( min(1.0, (w1 * f1 + w2 * f2) / w), min(MAX_CONFIDENCE, max(max(Truth_w2c(w), c1), c2)))

def Truth_Difference(v1, v2):
    ((f1,c1), (f2,c2)) = (v1, v2)
    return (f1 * (1.0 - f2), c1*c2)

def Truth_Expectation(T):
    (f,c) = T
    return (c * (f - 0.5) + 0.5)

#Positive evidence collection:
#Case 1: A and B share a common property C
#A --> C
#B --> C
#derive:
#w+ for A --> B

#Case 2: A and B share a common instance
#A --> C
#B --> C
#derive:
#w+ for A --> B

#Case 3: B has a property which B does not have
#B --> C
#not A --> C
#derive:
#w- for A --> B

#Case 4: A has an instance which B does not have
#C --> A
#not C --> B
#derive:
#w- for A --> B

class Nalifier:
    #identify instance as existing instance if matching better than this value, else create new one:
    SUFFICIENT_MATCH_EXP = 0.8 # 0.5 #sys.argv[1] if len(sys.argv) > 1 else 0.5 #0.5 #0.5   #how good match to the best case in order to not use the new instance ID
    SUFFICIENT_DIFFERENCE_EXP = 0.0 #how much difference there needs to be to describe the case as different than to the best matched one
    COMMON_PROPERTY_EXP = 0.5 #how similar properties need to be in order for them to be used for building new concepts
    InstanceCreation=True
    ConceptCreation=False
    RelativeComparison=False
    ClosedWorldAssumption=False
    UseIntensionalDifference=True
    usecounts = {}
    prototypes = {}
    position0 = {}
    position1 = {}
    conceptnames = set([])
    current_prototypes = {}
    last_instance = None
    last_winner = None
    last_winner_truth_exp = 0.0
    last_winner_reldata = None
    last_winner_common_properties = set([])
    last_label = None
    last_label_frequency = 0.5
    winner_match_asymmetric = False
    binary_extreme_comparison_properties = set([])
    continuous_comparison_properties = set([])
    label_properties = set([])
    Events = []
    concept_id = 1
    sensorValueReporters = dict([])
    conceptValueReporters = dict([])
    BestMatch = ""
    BiggestDifference = ""
    propertyOfInterest = None
    
    def __init__(self, AIKR_Limit = 10):
      self.AIKR_Limit = AIKR_Limit

    def removeInstanceProperties(self, worstproto):
      removals = []
      for key in self.conceptValueReporters.keys():
          if key.startswith(worstproto + "_"):
              removals.append(key)
      for removal in removals:
          del self.conceptValueReporters[removal]

    def differenceEvaluate(self, T1, T2, property, biggestDifferenceProp, biggestDifferenceTruth, term1, term2, relation, biggestDifferenceArg1, biggestDifferenceArg2, relativeDifference = None):
      continuous = property in self.continuous_comparison_properties
      truthDifference = Truth_Negation(Truth_FrequencyComparison(T1, T2)) if continuous else Truth_Difference(T1, T2)
      if relativeDifference is not None:
          truthDifference = relativeDifference
      #print("DIFFERENCE EVAL", property, T1,T2,Truth_Expectation(truthDifference))
      if (self.propertyOfInterest is None or property == self.propertyOfInterest) and Truth_Expectation(truthDifference) > Truth_Expectation(biggestDifferenceTruth):
        if T1[0] < T2[0]:
          relation = "-"
        else:
          relation = "+"
        biggestDifferenceProp = property
        biggestDifferenceTruth = truthDifference
        biggestDifferenceArg1 = term1
        biggestDifferenceArg2 = term2
      return biggestDifferenceProp, biggestDifferenceTruth, relation, biggestDifferenceArg1, biggestDifferenceArg2

    def inheritances(self, term1, terms_term1, term2, terms_term2, requireSameProperties=False, asymmetricComparison=True):
        Inheritances = dict([])
        biggestDifferenceProp = None
        biggestDifferenceTruth = (0.0, 1.0)
        biggestDifferenceArg1 = None
        biggestDifferenceArg2 = None
        rel = "+"
        commonProperties = set([])
        incomparable = []
        ((extension1, intension1), (extension2, intension2)) = (terms_term1, terms_term2)
        if term2 in self.conceptnames and not asymmetricComparison:
          return None, None, None
        if term2 not in self.conceptnames and asymmetricComparison:
          return None, None, None
        hadMissingProp = False
        w_plus = 0
        w_minus = 0
        truth2 = (0.5, 0.0)
        truth3 = (0.5, 0.0)
        f_Induction = Truth_Induction if asymmetricComparison else Truth_Comparison
        f_Abduction = Truth_Abduction if asymmetricComparison else Truth_Comparison
        for prop1, T1 in intension1:
            for prop2, T2 in intension2:
                if prop1 == prop2:
                    w_plus+=1
                    truthPlus = (1.0, 0.5)
                    truth2 = Truth_Revision(truth2, truthPlus)
                    truthIntermediate = f_Induction(T1, T2) if prop1 not in self.continuous_comparison_properties else Truth_FrequencyComparison(T1, T2)
                    if Truth_Expectation(truthIntermediate) > self.COMMON_PROPERTY_EXP:
                        commonProperties.add((prop2, Truth_Revision(T1, T2)))
                    truth3 = Truth_Revision(truth3, truthIntermediate)
                    instance_property = term2 + "_" + prop1
                    nodeRelativeDifference = None
                    if prop1 in self.continuous_comparison_properties and self.RelativeComparison and instance_property in self.conceptValueReporters:
                        #get the second truth value from the value reporter of term1_prop1
                        nodeRelativeDifference = Truth_Negation(Truth_FrequencyComparison((0.5, 0.9), self.conceptValueReporters[instance_property].reportValue(T1[0], Sensation_Reliance=T1[1], RangeUpdate=False, Print=False)))
                        #print("//RELATIVE VALUE GET", instance_property, T1, nodeRelativeDifference, self.conceptValueReporters[instance_property].values)
                    biggestDifferenceProp, biggestDifferenceTruth, rel, biggestDifferenceArg1, biggestDifferenceArg2 = self.differenceEvaluate(T1, T2, prop1, biggestDifferenceProp, biggestDifferenceTruth, term1, term2, rel, biggestDifferenceArg1, biggestDifferenceArg2, relativeDifference=nodeRelativeDifference)
        for prop1, T1 in extension1:
            for prop2, T2 in extension2:
                if prop1 == prop2:
                    w_plus+=1
                    truthPlus = (1.0, 0.5)
                    truth2 = Truth_Revision(truth2, truthPlus)
                    truth3 = Truth_Revision(truth3, f_Abduction(T1, T2))
        for prop2, T2 in intension2:
            AHasProperty = False
            for prop1, T1 in intension1:
                if prop1 == prop2:
                    AHasProperty = True
            if not AHasProperty and requireSameProperties:
                hadMissingProp = True
            if not AHasProperty and self.ClosedWorldAssumption and prop2 not in self.continuous_comparison_properties:
                w_minus+=1
                truthMinus = (0.0, 0.5)
                truth2 = Truth_Revision(truth2, truthMinus)
                T1 = (0.0, 1.0) #fabricated truth since the property does not appear in T1
                truth3 = Truth_Revision(truth3, f_Induction(T1, T2))
                #biggestDifferenceProp, biggestDifferenceTruth, rel, biggestDifferenceArg1, biggestDifferenceArg2 = differenceEvaluate(T1, T2, prop2, biggestDifferenceProp, biggestDifferenceTruth, term1, term2, rel, biggestDifferenceArg1, biggestDifferenceArg2)
                if prop2 in self.label_properties:
                    incomparable.append(term2)
        for inst1, T1 in extension1:
            BHasInstance = False
            for inst2, T2 in extension2:
                if inst1 == inst2:
                    BHasInstance = True
            if not BHasInstance and applyCWA:
                w_minus+=1
                truthMinus = (0.0, 0.5)
                truth2 = Truth_Revision(truth2, truthMinus)
                T2 = (0.0, 1.0) #fabricated truth since the property does not appear in T1
                truth3 = Truth_Revision(truth3, f_Abduction(T1, T2))
        if not (hadMissingProp and requireSameProperties):
            Inheritances[term2] = truth3
        for bad_term in incomparable:
            Inheritances.pop(bad_term, None)
        return Inheritances, (biggestDifferenceProp, biggestDifferenceTruth, rel, biggestDifferenceArg1, biggestDifferenceArg2), commonProperties

    def AddInput(self, inp, inverted=False, Print=False, Sensation_Reliance=0.9): #<{instance} --> [property]>. :|: %frequency%
        if inp.startswith("//"):
            print(inp)
            return
        if inp != "1":
            instance = inp.split("{")[1].split("}")[0]
            property = inp.split("[")[1].split("]")[0]
            if "|->" in inp:
                if property not in self.sensorValueReporters:
                  self.sensorValueReporters[property] = ValueReporter()
                self.continuous_comparison_properties.add(property)
            frequency = float(inp.split("%")[1].split("%")[0].split(";")[0]) if "%" in inp else 1.0
            if property == "position0":
              position0[instance] = frequency
              return
            if property == "position1":
              position1[instance] = frequency
              return
            if property.startswith("label_"):
              self.last_label = property.split("label_")[1]
              self.last_label_frequency = frequency
            if not inverted and property in self.binary_extreme_comparison_properties:
                NAL_AddInput("<{" + instance + "}" + " --> [" + "anti_" +  property + "]>. :|: %" + str(1-frequency) + "%", True, Print=Print)
        if inp == "1": # or (instance != self.last_instance and self.last_instance is not None):
            #DETERMINE BEST MATCH IN CURRENT PROTOTYPES:
            if self.last_winner is not None and self.last_winner_truth_exp > self.SUFFICIENT_MATCH_EXP:
                for k,v in self.last_winner.items(): #just 1
                    if k not in self.usecounts:
                      self.usecounts[k] = 1
                    else:
                      self.usecounts[k] = self.usecounts[k] + 1
                    (biggestDifferenceProp, biggestDifferenceTruth, rel, biggestDifferenceArg1, biggestDifferenceArg2) = self.last_winner_reldata
                    if Truth_Expectation(biggestDifferenceTruth) > self.SUFFICIENT_DIFFERENCE_EXP:
                      reduced_instance_representation = self.last_instance
                      #USE VARIABLE INSTEAD OF 2 STATEMENTS!!!!
                      termname = lambda T: "{" + T + "}" if T not in self.conceptnames else T
                      inst1 = termname(reduced_instance_representation) #"#1"
                      inst2 = termname(k)
                      inst1forRel = termname(biggestDifferenceArg1) if rel == "+" else termname(biggestDifferenceArg2)
                      inst2forRel = termname(biggestDifferenceArg2) if rel == "+" else termname(biggestDifferenceArg1)
                      evP = None
                      self.BiggestDifference = (rel, biggestDifferenceProp)
                      rel = "--> " if self.winner_match_asymmetric else "<->"
                      self.BestMatch = (rel, k)
                      #print("//BEST MATCH!!!", self.BestMatch, self.BiggestDifference)
                      if self.RelativeComparison:
                          for props in self.current_prototypes[list(self.current_prototypes.keys())[0]]:
                              for prop in props:
                                instance_property = k + "_" + prop[0]
                                frequency = prop[1][0]
                                if instance_property not in self.conceptValueReporters:
                                    self.conceptValueReporters[instance_property] = ValueReporter()
                                self.conceptValueReporters[instance_property].reportValue(frequency, Print=False, Sensation_Reliance=prop[1][1])
                                #print("//RELATIVE VALUE SET", instance_property, self.conceptValueReporters[instance_property].reportValue(frequency, RangeUpdate=False, Print=False, Sensation_Reliance=prop[1][1]))
                      relStatement = f"<({inst1forRel} * {inst2forRel}) --> (+ {biggestDifferenceProp})>"
                      if self.UseIntensionalDifference and biggestDifferenceProp not in self.continuous_comparison_properties:
                          relStatement = f"<({inst1forRel} ~ {inst2forRel}) --> [{biggestDifferenceProp}]>"
                      if self.last_label is not None: 
                        #ev1 = f"((<{{{inst2}}} <-> {inst1}> && {relStatement}) && <{{{inst2}}} --> {self.last_label}>). :|: %{self.last_label_frequency};{0.9}%"
                        ev1 = f"(<{inst1forRel} {rel} {inst2forRel}> && {relStatement}). :|: %{self.last_label_frequency};{0.9}%"
                      else:
                        ev1 = f"(<{inst1forRel} {rel} {inst2forRel}> && {relStatement}). :|:"
                      inst1 = inst1.replace("{","").replace("}","") #not elegant
                      inst2 = inst2.replace("{","").replace("}","") #todo improve
                      if inst1 in self.position0 and inst2 in self.position0 and inst1 in self.position1 and inst2 in self.position1 and not self.winner_match_asymmetric:
                          if self.position0[inst1] > self.position0[inst2]:
                            if position1[inst1] > position1[inst2]:
                              evP = f"(<({{{inst1}}} * {{{inst2}}}) --> (+ position0)> && <({{{inst1}}} * {{{inst2}}}) --> (+ position1)>). :|: %1.0;0.6%"
                            else:
                              evP = f"(<({{{inst1}}} * {{{inst2}}}) --> (+ position0)> && <({{{inst2}}} * {{{inst1}}}) --> (+ position1)>). :|: %1.0;0.6%"
                          else:
                            if self.position1[inst1] > self.position1[inst2]:
                              evP = f"(<({{{inst2}}} * {{{inst1}}}) --> (+ position0)> && <({{{inst1}}} * {{{inst2}}}) --> (+ position1)>). :|: %1.0;0.6%"
                            else:
                              evP = f"(<({{{inst2}}} * {{{inst1}}}) --> (+ position0)> && <({{{inst2}}} * {{{inst1}}}) --> (+ position1)>). :|: %1.0;0.6%"
                      self.Events.append(ev1)
                      if evP is not None:
                        if Print:
                          print(evP)
                        self.Events.append(evP)
                      if Print:
                        print(ev1)
                    else:
                      if self.last_label is not None:
                        ev2 = f"<{{{k}}} --> {self.last_label}>. :|: %{self.last_label_frequency};{0.9}%"
                      else:
                        ev2 = f"<{{{k}}} --> [see]>. :|:"
                      self.Events.append(ev2)
                      if Print:
                        print(ev2)
                      break
            if self.last_winner is not None:
                #ADD NEW CONCEPT NODE IF THE MATCH WAS BASED ON INSTANCE (symmetric) COMPARISON
                names = sorted([k for (k,v) in self.last_winner_common_properties])
                conceptname = "_".join(names)
                #print(conceptname); exit(0);
                if self.ConceptCreation and conceptname != "" and not self.winner_match_asymmetric:
                    conceptname += "_" + str(self.concept_id)
                    self.concept_id += 1
                    self.conceptnames.add(conceptname)
                    print("//CONCEPT CREATION", conceptname, self.last_winner_common_properties)
                    self.prototypes[conceptname] = (set([]), self.last_winner_common_properties)
            #else:
            if self.winner_match_asymmetric or not (self.last_winner is not None and self.last_winner_truth_exp > self.SUFFICIENT_MATCH_EXP):
              if self.last_winner is None:
                if self.last_label is not None:
                  ev3 = f"<{{{self.last_instance}}} --> {self.last_label}>. :|: %{self.last_label_frequency};{0.9}%"
                else:
                  ev3 = f"<{{{self.last_instance}}} --> [see]>. :|:"
                self.Events.append(ev3)
                if Print:
                  print(ev3)
              if self.InstanceCreation:
                self.prototypes.update(self.current_prototypes)
            self.current_prototypes = {}
        if inp == "1":
            self.propertyOfInterest = None
            if self.last_instance not in self.prototypes:
                self.removeInstanceProperties(self.last_instance)
            return
        self.last_instance = instance
        if Print:
          print("//" + inp)
        if instance not in self.current_prototypes:
            self.current_prototypes[instance] = (set(),set())
        #check if instance is in prototypes, in which case the truth value is the revised one:
        RevisedInstanceProperty = False
        if instance in self.prototypes:
            for (prop, TV) in self.prototypes[instance][1]:
                if prop == property:
                    self.current_prototypes[instance][1].add((property, Truth_Revision((frequency, 0.9), TV)))
                    RevisedInstanceProperty = True
                    break
        if not RevisedInstanceProperty:
            self.current_prototypes[instance][1].add((property, (frequency, 0.9)))
        if self.RelativeComparison:
            instance_property = instance+"_"+property
            if instance_property not in self.conceptValueReporters:
                self.conceptValueReporters[instance_property] = ValueReporter()
            self.conceptValueReporters[instance_property].reportValue(frequency, Print=False, Sensation_Reliance=Sensation_Reliance)
            #print("//RELATIVE VALUE INIT", instance_property, self.conceptValueReporters[instance_property].reportValue(frequency, RangeUpdate=False, Print=False, Sensation_Reliance=Sensation_Reliance))
        winner = None
        winner_truth_exp = 0.0
        for (key, value) in self.prototypes.items():
            candidate, reldata, commonProperties = self.inheritances(instance, self.current_prototypes[instance], key, value, asymmetricComparison=False)
            if candidate is not None and list(candidate.values()):
              candidate_truth_exp = Truth_Expectation(list(candidate.values())[0])
              if candidate_truth_exp > winner_truth_exp:
                  winner = candidate
                  winner_truth_exp = candidate_truth_exp
                  self.last_winner_reldata = reldata
                  self.last_winner_common_properties = commonProperties
                  self.winner_match_asymmetric = False
        if winner_truth_exp <= self.SUFFICIENT_MATCH_EXP:
          for (key, value) in self.prototypes.items():
            candidate, reldata, commonProperties = self.inheritances(instance, self.current_prototypes[instance], key, value, asymmetricComparison=True)
            if candidate is not None and list(candidate.values()):
              candidate_truth_exp = Truth_Expectation(list(candidate.values())[0])
              if candidate_truth_exp > winner_truth_exp:
                  winner = candidate
                  winner_truth_exp = candidate_truth_exp
                  self.last_winner_reldata = reldata
                  self.last_winner_common_properties = commonProperties
                  self.winner_match_asymmetric = True
        self.last_winner_truth_exp = winner_truth_exp
        self.last_winner = winner
        #capacity limit exceeded
        if len(self.prototypes) > self.AIKR_Limit:
          minuse = 99999999
          worstproto = None
          for proto in self.prototypes:
            usecount = self.usecounts[proto] if proto in self.usecounts else 0
            if usecount < minuse:
              minuse = usecount
              worstproto = proto
          self.usecounts.pop(worstproto, None)
          self.prototypes.pop(worstproto, None)
          self.position0.pop(worstproto, None)
          self.position1.pop(worstproto, None)
          self.conceptnames.discard(worstproto)
          self.removeInstanceProperties(worstproto)

    def AddInputVector(self, name, values, dimname=None, Print=False, UseHistogram=True, Sensation_Reliance = 0.9):
        global sensorValueReporters
        if dimname is None:
          dimname = name
        for i, value in enumerate(values):
            propertyName = dimname + str(i)
            if UseHistogram:
                if propertyName not in self.sensorValueReporters:
                  self.sensorValueReporters[propertyName] = ValueReporter()
                #binary_extreme_comparison_properties.add(propertyName)
                self.continuous_comparison_properties.add(propertyName)
                (f,c) = self.sensorValueReporters[propertyName].reportValue(value, Print=False, RangeUpdate=self.InstanceCreation, Sensation_Reliance = Sensation_Reliance)
            else:
                (f,c) = (value, Sensation_Reliance)
            self.AddInput("<{" + name + "} |-> [" + propertyName + "]>. :|: %" + str(f) + "%", Print=Print, Sensation_Reliance=Sensation_Reliance) # + str(c) + "%")

    def ShellInput(self, inp):
        if inp.startswith("*SET_CONTINUOUS="):
            propertyName = inp.split("*SET_CONTINUOUS=")[1]
            if propertyName not in self.sensorValueReporters:
              self.sensorValueReporters[propertyName] = ValueReporter()
            self.continuous_comparison_properties.add(propertyName)
            return None
        if inp.startswith("*PROTOTYPES"):
            print("//"+str(self.prototypes))
            return None
        if inp.startswith("*PROPERTY_OF_INTEREST="):
            self.propertyOfInterest = inp.split("*PROPERTY_OF_INTEREST=")[1]
            if self.propertyOfInterest == "":
                self.propertyOfInterest = None
            return None
        if inp.startswith("*RESET_PROTOTYPES="):
            self = Nalifier(int(inp.split("*RESET_PROTOTYPES=")[1]))
            return None
        if inp.startswith("*SUFFICIENT_MATCH_EXP="):
            self.SUFFICIENT_MATCH_EXP = float(inp.split("*SUFFICIENT_MATCH_EXP=")[1])
            return None
        if inp.startswith("*SUFFICIENT_DIFFERENCE_EXP="):
            self.SUFFICIENT_DIFFERENCE_EXP = float(inp.split("*SUFFICIENT_DIFFERENCE_EXP=")[1])
            return None
        if inp.startswith("*COMMON_PROPERTY_EXP="):
            self.COMMON_PROPERTY_EXP = float(inp.split("*COMMON_PROPERTY_EXP=")[1])
            return None
        if inp.startswith("*CONCEPT_CREATION="):
            self.ConceptCreation = True if inp.split("*CONCEPT_CREATION=")[1].lower() == "true" else False
            return None
        if inp.startswith("*INSTANCE_CREATION="):
            self.InstanceCreation = True if inp.split("*INSTANCE_CREATION=")[1].lower() == "true" else False
            return None
        if inp.startswith("*RELATIVE_COMPARISON="):
            self.RelativeComparison = True if inp.split("*RELATIVE_COMPARISON=")[1].lower() == "true" else False
            return None
        if inp.startswith("*CLOSED_WORLD_ASSUMPTION="):
            self.ClosedWorldAssumption = True if inp.split("*CLOSED_WORLD_ASSUMPTION=")[1].lower() == "true" else False
            return None
        if inp.startswith("*USE_INTENSIONAL_DIFFERENCE="): #will be used for non-continuous properties only even when true
            self.UseIntensionalDifference = True if inp.split("*USE_INTENSIONAL_DIFFERENCE=")[1].lower() == "true" else False
            return None
        lhs = inp.split(". :|:")[0]
        if inp == "1":
            self.AddInput(inp, Print=True)
            return None
        elif ". :|:" in inp and "{" in inp and "}" in inp and "[" in inp and "]" in inp and " * " not in inp:
            self.AddInput(inp, Print=True)
            return None
        else:
            return inp

if "test" in sys.argv:
    nalifier = Nalifier(10)
    nalifier.ConceptCreation = False
    Print = False
    nalifier.SUFFICIENT_MATCH_EXP = 1.0 #create new node for each entry
    nalifier.AddInputVector("red", [255, 0, 0], dimname="rgb", Print=Print)
    nalifier.AddInput("1", Print=Print)
    nalifier.AddInputVector("green", [0, 255, 0], dimname="rgb", Print=Print)
    nalifier.AddInput("1", Print=Print)
    nalifier.AddInputVector("blue", [0, 0, 255], dimname="rgb", Print=Print)
    nalifier.AddInput("1", Print=Print)
    nalifier.InstanceCreation = False
    #test:
    nalifier.AddInputVector("newcolor1", [0, 0, 5], dimname="rgb", Print=True)
    nalifier.Events = [] #we aren't interested in previous events
    nalifier.SUFFICIENT_MATCH_EXP = 0.0 #find nearest node
    nalifier.AddInput("1", Print=True)
    print(nalifier.prototypes)
    print(nalifier.BestMatch)
    print(nalifier.BiggestDifference)
    exit(0)

if __name__ == "__main__":
    nalifier = Nalifier(10)
    while True:
        try:
            inp = input().rstrip("\n")
        except:
            exit(0)
        ret = nalifier.ShellInput(inp)
        if ret is not None:
            print(ret)
