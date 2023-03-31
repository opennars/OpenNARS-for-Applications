import NAR
import time
import random
import math

NAR.AddInput("*volume=0")
def distance(posX, posY, posX2, posY2):
    dx = posX - posX2
    dy = posY - posY2
    return max(abs(dx), abs(dy))

class Street:
    def __init__(self,forCarsOnly, startX, startY, endX, endY):
        self.startX = startX;
        self.startY = startY;
        self.endX = endX;
        self.endY = endY;
        self.forCarsOnly = forCarsOnly;

class Entity:
    entityID = 0
    label = "";
    posX = 0.0
    posY = 0.0
    width = 1.0
    height = 1.0
    velocity = 0.0
    angle = 0.0
    id = 0
    scale = 1.0
    maxSpeed = 2.0
    pedestrianIgnoreTrafficLight = False
    carIgnoreTrafficLight = False
    isPredicted = False
    lastPosX = 0
    lastPosY = 0
    def __init__(self, id, posX, posY, velocity, angle):
        self.id = id
        self.posX = posX
        self.posY = posY
        self.velocity = velocity
        self.angle = angle

    def hasMoved(self):
        dist = self.velocity;
        return dist >= 0.1

    def simulate(self, trafficLights, entities, streets):
        accelerate = True;
        for l in trafficLights:
            if distance(self.posX, self.posY, l.posX, l.posY) < l.radius:
                if l.colour == l.RED:
                    if random.uniform(0,1) > 0.3 and ((isinstance(self,Car) and not self.carIgnoreTrafficLight) or (isinstance(self, Pedestrian) and not self.pedestrianIgnoreTrafficLight)):
                        self.velocity *= 0.5;
                        accelerate = False
        for e in entities:
            collidable = not (isinstance(self,Pedestrian) and isinstance(e,Pedestrian))
            if e != self and collidable:
                nearEnough = 10
                k = 0
                while k<nearEnough:
                    k += 0.1
                    pXNew = self.posX + k * math.cos(self.angle)
                    pYNew = self.posY + k * math.sin(self.angle)
                    if distance(pXNew, pYNew, e.posX, e.posY) < nearEnough:
                        self.velocity *= 0.8
                        accelerate = False
        if accelerate and self.velocity < self.maxSpeed:
            self.velocity += 0.02
        aX = math.cos(self.angle)
        aY = math.sin(self.angle)
        self.posX += aX * self.velocity
        self.posY += aY * self.velocity
        epsilon = 1
        if self.posY < 0:
            self.posY = 1000 - epsilon
            #this.id = entityID++;
        if self.posY > 1000:
            self.posY = epsilon
            #this.id = entityID++;
        if self.posX < 0:
            self.posX = 1000 - epsilon
            #this.id = entityID++;
        if self.posX > 1000:
            self.posX = epsilon
            #this.id = entityID++;


class Pedestrian(Entity):
    prevX = 0
    prevY = 0
    def __init__(self, id, posX, posY, velocity, angle):
        super().__init__(id, posX, posY, velocity, angle)
        self.initialAngle = angle
        self.maxSpeed = 1
    def simulate(self, trafficLights, entities, streets):
        super().simulate(trafficLights, entities, streets)
        self.angle+=(random.uniform(0,1)*0.1-0.05)
        #ok pedestrian, don't go on grass
        forPedestrians = False
        for street in streets:
            if not street.forCarsOnly and self.posX > street.startX and self.posX < street.endX and self.posY > street.startY + discretization and self.posY < street.endY:
                forPedestrians = True
                break
        if not forPedestrians:
            self.angle = self.initialAngle
            self.posX = self.prevX
            self.posY = self.prevY
        self.prevX = self.posX
        self.prevY = self.posY

class Car(Entity):
    maxSpeed = 2

    
def positionToTerm(X, Y, discretization):
    posX = int(X / discretization)
    posY = int(Y / discretization)
    return str(posX) + "_" + str(posY)

discretization = 10

def entityname(ent):
    C = "C" if isinstance(ent, Car) else "P"
    return C+str(ent.id)

class InformPredictionNar:
    lastInput = "";
    input = "";
    inputs = []
    #minX and minY define the lower end of the relative coordinate system
    def informAboutEntity(self, ent, minX, minY):
        id = str(ent.id)
        useMultipleIDs = True
        if not useMultipleIDs:
            id = "0"
        pos = positionToTerm(int(ent.posX-minX), int(ent.posY-minY), discretization);
        self.inputs.append("(*,"+entityname(ent) + ","+ pos + "). :|:");
        self.input += self.inputs[-1]

    def informAboutTrafficLight(self, light, minX, minY):
        colour = "green" if light.colour == 0 else "red"
        narsese = colour + ". :|:" #"<trafficLight --> ["+colour+"]>. :|:"
        self.inputs.append(narsese);
        self.input+=narsese;

    #param force are the inputs forced to be fed into the reasoner
    def Input(self, force):
        hadInput = False
        if self.input != self.lastInput or force:
            for inp in self.inputs:
                NAR.AddInput(inp)
                hadInput = True
            self.lastInput = self.input
        self.input = "";
        self.inputs = []
        return hadInput

class Camera:
    radius=48
    viewradius = 50
    def __init__(self, posX, posY):
        self.posX = posX;
        self.posY = posY;
        self.minX = posX-self.radius*2;
        self.minY = posY-self.radius*2;
        self.informer = InformPredictionNar();
    
    def see(self, entities, trafficLights, force):
        for ent in entities:
            if distance(self.posX, self.posY, ent.posX, ent.posY) < self.viewradius:
                self.informer.informAboutEntity(ent, self.minX, self.minY)
        for ent in trafficLights:
            if ent.switched: # and distance(self.posX, self.posY, ent.posX, ent.posY) < self.viewradius:
                self.informer.informAboutTrafficLight(ent, self.minX, self.minY)
                break
        return self.informer.Input(force);

class TrafficLight:
    GREEN = 0
    RED = 1
    t = 1
    switched = False
    def __init__(self, id, radius, positionX, positionY, colour):
        self.radius = radius
        self.id = id
        self.posX = positionX
        self.posY = positionY
        self.colour = colour
    def simulate(self, trafficLights, entities, streets):
        self.switched = False
        if self.t % 200 == 0:
            self.colour = (self.colour + 1) % 2;
            self.switched = True
        self.t+=1

streets = []
trafficLights = []
entities = []
cameras = []

streetWidth = 40
trafficLightRadius = 25;
cameras.append(Camera(500+streetWidth/2, 500+streetWidth/2));
streets.append(Street(False, 0, 500, 1000, 500 + streetWidth));
streets.append(Street(True, 500, 0, 500 + streetWidth, 1000));
trafficLights.append(TrafficLight(1, trafficLightRadius, 500 + streetWidth + trafficLightRadius, 500 + streetWidth/2, 0))
trafficLights.append(TrafficLight(2, trafficLightRadius, 500 - trafficLightRadius, 10+ 500 + streetWidth/2, 0))
trafficLights.append(TrafficLight(3, trafficLightRadius/2, 500 + streetWidth, 500 + streetWidth + trafficLightRadius, 1))
trafficLights.append(TrafficLight(4, trafficLightRadius/2, 10+500, 500 - trafficLightRadius, 1))
cars = 4 #cars and pedestrians
i=0.0
entityID = 1
while i < cars/2:
    entities.append(Car(entityID, 500 + streetWidth - discretization, 900 - i * 100, 0.3, -math.pi / 2));
    entityID+=1
    entities.append(Car(entityID, 500 + 2*discretization, 900 - i * 100, 0.3, math.pi / 2));
    entityID+=1
    break
    i += 1.05
pedestrians = 4
i=0.0
while i < pedestrians/2:
    entities.append(Pedestrian(entityID, 900 - i * 100, 500 + streetWidth - 1, 0.3, 0));
    entityID+=1
    entities.append(Pedestrian(entityID, 900 - i * 100, 500 + discretization*2, 0.3, -math.pi));
    entityID+=1
    break
    i += 1.05

def cycle():
    hadInput = False
    for c in cameras:
        force = True
        #hadInput = hadInput or c.see(entities, trafficLights, force)
        c.see(entities, trafficLights, force)
    for e in entities + trafficLights:
        e.simulate(trafficLights, entities, streets)

def streetcolor(x,y):
    for street in streets:
        if x>=street.startX and x<=street.endX and y>=street.startY and y<=street.endY:
            if distance(x, y, cameras[0].posX, cameras[0].posY) < cameras[0].viewradius:
                return "\x1b[47m"
            return "\x1b[43m" 
    return ""

def drawIDtoDirectionIndicator(drawid):
    if drawid == "1":
        drawid = "^"
    if drawid == "2":
        drawid = "v"
    if drawid == "3":
        drawid = ">"
    if drawid == "4":
        drawid = "<"
    return drawid

for i in range(100000):
    cycle()
    drawsizeX = 20 #cameras[0].radius
    drawsizeY = 20 #cameras[0].radius
    drawings=[["  " for i in range(drawsizeX)] for j in range(drawsizeY+1)]
    occurrenceTimeQuery = NAR.AddInput("?1? :\:", Print=False)["answers"][0]
    incolor = "\x1B[31m"
    predcolor = "\x1B[36m"
    colorend = "\x1B[0m"
    #NAR.AddInput("1")
    for x in range(drawsizeX):
        for y in range(drawsizeY):
            streetcol = streetcolor(x*discretization+cameras[0].minX,y*discretization+cameras[0].minY)
            drawings[x][y] = streetcol + "  " + colorend
    if occurrenceTimeQuery["term"] != 'None':
        lastinputocc = int(occurrenceTimeQuery["occurrenceTime"])
        for x in range(drawsizeX):
            for y in range(drawsizeY):
                loc = f"{x}_{y}"
                ret = NAR.AddInput("(?1 * " + loc + ")? :/:", Print=False)
                if ret["answers"][0]["term"] != 'None':
                    answer = ret["answers"][0]
                    answerocc = int(answer["occurrenceTime"])
                    f,c = float(answer["truth"]["frequency"]), float(answer["truth"]["confidence"])
                    truthexp = (c * (f - 0.5) + 0.5)
                    color = predcolor
                    if answerocc > lastinputocc and truthexp > 0.51:# and answerocc - lastinputocc < 5:
                        if answerocc - lastinputocc > 15:
                            predcolor = "\x1b[34m"
                        answerterm = answer["term"]
                        entname = answerterm.split("(")[1].split(" * ")[0]
                        streetcol = streetcolor(x*discretization+cameras[0].minX,y*discretization+cameras[0].minY)
                        drawings[y][x] = streetcol + predcolor + entname[0] + drawIDtoDirectionIndicator(entname[1]) + colorend
        for ent in entities+trafficLights:
            loc = [int(x) for x in positionToTerm(ent.posX-cameras[0].minX, ent.posY-cameras[0].minY, discretization).split("_")]
            x,y = loc[0], loc[1]
            if x >= 0 and y >= 0 and x < drawsizeX and y < drawsizeY:
                streetcol = ""
                drawchar = 'C'
                drawid = str(ent.id)
                color = incolor
                if isinstance(ent, Pedestrian):
                    drawchar="P"
                if isinstance(ent, TrafficLight):
                    drawid = " "
                    if ent.colour == ent.GREEN:
                        drawchar="G"
                        color = "\x1b[32;42m"
                    if ent.colour == ent.RED:
                        drawchar="R"
                        color = "\x1b[31;41m"
                else:
                    streetcol = streetcolor(ent.posX,ent.posY)
                drawings[y][x] = streetcol + color + drawchar + drawIDtoDirectionIndicator(drawid) + colorend
    print("\033[1;1H\033[2J")
    for line in drawings:
        print("".join(line))
