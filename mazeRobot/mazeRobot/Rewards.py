class Rewards:
    def __init__(self, simNode):
        self.simNode = simNode
        self.erroneousSample = False

    def hasHitWall(self):
        straightDistMin = 0.16
        sideDistMin = 0.16
        diagDistMin = 0.16  

        if ((self.simNode.frontSensor < straightDistMin) or (self.simNode.backSensor < straightDistMin) or (self.simNode.leftSensor < sideDistMin) or (self.simNode.rightSensor < sideDistMin) or (self.simNode.backleftSensor < diagDistMin) 
        or (self.simNode.backrightSensor < diagDistMin) or (self.simNode.frontleftSensor < diagDistMin) or (self.simNode.frontrightSensor < diagDistMin)):  # FIXME Tinker
            return True
        else:
            return False

    def hasReachedEnd(self):
        if self.simNode.frontSensor >= 40:    #FIXME make a 15 meter long outer edge to disignate as the endpoint
            return True
        else:
            return False

    def calculateReward(self):
        if (self.simNode.curAction == 2 or self.simNode.curAction == 3):
            actionIsTurn = True
        else:
            actionIsTurn = False

        r = 0

        # Punishments 
        if self.hasHitWall():               
            r -= 100
        if self.simNode.movedaway:
            r -= 5
        if (self.simNode.didOscillate) and (not self.simNode.steeredWell) and (not self.simNode.closerToWaypnt):
            r -= 1
        if (actionIsTurn) and (not self.simNode.steeredWell):  
            r -= 5
        if (self.simNode.goodAim) and (self.simNode.movedaway):
            r -= 9
        

        # Rewards
        if self.simNode.closerToWaypnt and self.simNode.curAction == 1:
            r += 3
        if self.simNode.goodAim and self.simNode.closerToWaypnt:
            r += 9
        if self.simNode.steeredWell:
            r += 4
        return r 
