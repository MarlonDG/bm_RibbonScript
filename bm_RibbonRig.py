import pymel.core as pm
import pprint

class bm_RibbonRig(object):

    def __init__(self):

        #curve = self.curveFilter()
        #self.surfaceCreation(curve)



    # --------------------------------------------------
    # filtering the selection must be curve
    # --------------------------------------------------

    def curveFilter(self):

        selection = pm.ls(sl=1)[0]

        crvShape = selection.getShape()

        if crvShape.type() == 'nurbsCurve':

            return crvShape

        else:
            pm.warning('     Select a Curve to continue..............     ')

    # --------------------------------------------------
    # create and orient the joints
    # --------------------------------------------------
    def jointCreation(self, curve):

        name = 'Ribbon_Bind_'

        curveCVs = curve.getCVs()

        i = 0

        jntList = []

        # Joint creation per cv
        for x in curveCVs:

            pm.select(cl=1)

            i += 1
            ribbonJnt = pm.joint(n=name + str(i))
            ribbonJnt.translate.set(x)

            # Call the method to create a PMA and LT nodes per bind Joint
            self.bindJntNodelling(ribbonJnt)

            jntList.append(ribbonJnt)

        # Parenting joints
        for j in range(1, len(jntList)):

            pm.parent(jntList[j], jntList[j-1])

        baseJoint = jntList[0]

        pm.joint(baseJoint, e=True, oj='xyz', secondaryAxisOrient='yup', ch=True, zso=True)

        endJoint = jntList[-1]

        endJoint.jointOrientX.set(0)
        endJoint.jointOrientY.set(0)
        endJoint.jointOrientZ.set(0)

        pm.select(cl=1)

        return jntList

    # --------------------------------------------------
    # create the ribbon surface and pointOnSurface Joints
    # --------------------------------------------------
    def surfaceCreation(self, curve):

        bindJntChain = self.jointCreation(curve)


        loftCurves = []

        i = -1

        for x in range(2):

            jntPosList = []

            chain = pm.duplicate(bindJntChain)
            chain[0].translateZ.set(i)
            i += 2

            # Getting the children of the joint chains
            for k in chain:
                jntChildPos = k.getTranslation(space='world')
                jntPosList.append(jntChildPos)

            loftCurve = pm.curve(n='curve_' + str(x), degree=1, point=jntPosList)

            loftCurves.append(loftCurve)

            pm.delete(chain)

        surface = pm.loft(loftCurves[0], loftCurves[1], n='Ribbon_Surface', ar=True, reverseSurfaceNormals=False, degree=1, ss=1, object=True, ch=False, polygon=0)
        surface = pm.rebuildSurface(surface, ch=0, rpo=1, rt=0, end=1, kr=2, kcp=0, kc=0, su=0, du=3, sv=0, dv=3, tol=0.01, fr=0, dir=2 )

        pm.delete(surface, ch=True)

        for j in loftCurves:
            pm.delete(j)

        # Unparenting joints and duplicate joints for skin
        bindJntList = []
        skinJntList = []

        for b in range(len(bindJntChain)):
            pm.parent(bindJntChain[b], w=True)
            bindJnt = pm.PyNode(bindJntChain[b])
            bindJntList.append(bindJnt)

            #Here we add the method to nodelling

            if b == len(bindJntChain)-1:
                for l in range(len(bindJntChain)):
                    name = bindJntChain[l].replace('Bind', 'Skin')
                    skinJnt = pm.duplicate(bindJntChain[l], name=name)
                    skinJntList.append(skinJnt)

        # Skinning the surface to the bind joint
        pm.skinCluster(surface, bindJntList, tsb=True, skinMethod=0, maximumInfluences=1, dropoffRate=10.0)

        # Creating all the nodes to for the skin Joints
        surfaceShape = surface[0].getShape()

        for r in range(len(skinJntList)):

            skinJnt = skinJntList[r]

            distance = (1.0 / (len(skinJntList)-1)) * r

            self.skinJntControl(skinJnt, surfaceShape, distance)





    # --------------------------------------------------
    # create the nodes, controls for the skin Joints
    # --------------------------------------------------
    def skinJntControl(self, currentJoint, surface, distance):

        # Cube Shape Control
        name = currentJoint[0].replace('Skin', 'MicroCtrl')
        cubeControl = pm.curve(n=name, degree=1, knot=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16],
                                        point=[(-0.5, 0.5, -0.5), (-0.5, 0.5, 0.5), (0.5, 0.5, 0.5), (0.5, 0.5, -0.5), (-0.5, 0.5, -0.5),
                                                (-0.5, -0.5, -0.5), (-0.5, -0.5, 0.5), (0.5, -0.5, 0.5), (0.5, 0.5, 0.5), (-0.5, 0.5, 0.5),
                                                (-0.5, -0.5, 0.5), (-0.5, -0.5, -0.5), (0.5, -0.5, -0.5), (0.5, 0.5, -0.5), (0.5, 0.5, 0.5),
                                                (0.5, -0.5, 0.5), (0.5, -0.5, -0.5)])
        # Changing controls color
        microCtrlShape = cubeControl.getShape()
        microCtrlShape.overrideEnabled.set(1)
        microCtrlShape.overrideColor.set(17)

        ctrlOffsetGrp = pm.group(cubeControl, n=name + '_OffsetGrp')

        # Nodelling
        PoS_Node = pm.shadingNode('pointOnSurfaceInfo', asUtility=True, n=currentJoint[0].replace('Skin', 'PoS'))
        FbFMatrix_Node = pm.shadingNode('fourByFourMatrix', asUtility=True, n=currentJoint[0].replace('Skin', 'FbF_Mat'))
        decompMatrix_Node = pm.shadingNode('decomposeMatrix', asUtility=True, n=currentJoint[0].replace('Skin', 'Decomp_Mat'))

        # Connecting
        channels = ['X', 'Y', 'Z']

        surface.worldSpace.connect(PoS_Node.inputSurface)

        for x in range(len(channels)):
            channel = channels[x]

            pm.connectAttr(PoS_Node + '.normalizedNormal' + str(channel), FbFMatrix_Node + '.in0' + str(x))
            pm.connectAttr(PoS_Node + '.normalizedTangentU' + str(channel), FbFMatrix_Node + '.in1' + str(x))
            pm.connectAttr(PoS_Node + '.normalizedTangentV' + str(channel), FbFMatrix_Node + '.in2' + str(x))
            pm.connectAttr(PoS_Node + '.position' + str(channel), FbFMatrix_Node + '.in3' + str(x))

        FbFMatrix_Node.output.connect(decompMatrix_Node.inputMatrix)

        decompMatrix_Node.outputTranslate.connect(ctrlOffsetGrp.translate)
        decompMatrix_Node.outputRotate.connect(ctrlOffsetGrp.rotate)
        decompMatrix_Node.outputScale.connect(ctrlOffsetGrp.scale)

        # Setting
        PoS_Node.parameterU.set(0.5)

        PoS_Node.parameterV.set(distance)

        PoS_Node.turnOnPercentage.set(1)

        pm.parent(currentJoint, cubeControl)

    def bindJntNodelling(self, currentJnt):

        # Node Creation
        translateSum = pm.createNode('plusMinusAverage', n=currentJnt + 'transSum_PMA')
        #scaleInput = pm.createNode('layeredTexture', n=name.replace('_Trans','_Scale'))

        # Connecting
        translateSum.output3Dy.connect(currentJnt.translateY)
        translateSum.output3Dz.connect(currentJnt.translateZ)

        #scaleInput.outColor.connect(currentCtrl.scale)

    def macroControl(self, surface):

        numCtrl = 5

        ctrlList = []

        for c in range(numCtrl):
            distance = (1.0 / (numCtrl-1)) * c

            macroCtrl = pm.circle(normal=[1, 0, 0], n='Ribbon_Ctrl_' + str(c), r=3)
            ctrlList.append(macroCtrl)

            ctrlShape = macroCtrl[0].getShape()
            ctrlShape.overrideEnabled.set(1)
            ctrlShape.overrideColor.set(18)

            # Adding the offset for the controls
            ctrlOffset = pm.group(macroCtrl, n='Ribbon_Offset_' + str(c))

            # Nodelling
            PoS_Node = pm.shadingNode('pointOnSurfaceInfo', asUtility=True)
            FbFMatrix_Node = pm.shadingNode('fourByFourMatrix', asUtility=True)
            decompMatrix_Node = pm.shadingNode('decomposeMatrix', asUtility=True)

            channels = ['X', 'Y', 'Z']

            surface.worldSpace.connect(PoS_Node.inputSurface)


            for x in range(len(channels)):
                channel = channels[x]

                pm.connectAttr(PoS_Node + '.normalizedNormal' + str(channel), FbFMatrix_Node + '.in0' + str(x))
                pm.connectAttr(PoS_Node + '.normalizedTangentU' + str(channel), FbFMatrix_Node + '.in1' + str(x))
                pm.connectAttr(PoS_Node + '.normalizedTangentV' + str(channel), FbFMatrix_Node + '.in2' + str(x))
                pm.connectAttr(PoS_Node + '.position' + str(channel), FbFMatrix_Node + '.in3' + str(x))

                FbFMatrix_Node.output.connect(decompMatrix_Node.inputMatrix)

                decompMatrix_Node.outputTranslate.connect(ctrlOffset.translate)
                decompMatrix_Node.outputRotate.connect(ctrlOffset.rotate)
                #decompMatrix_Node.outputScale.connect(ctrlOffset.scale)

            # Setting
            PoS_Node.parameterU.set(0.5)

            PoS_Node.parameterV.set(distance)

            PoS_Node.turnOnPercentage.set(1)


            #Calling 'ctrlNode' method to create individual nodes for each ctrl
            #self.ctrlNode(ctrl[0], ctrlDistance)

            #ctrlList.append(ctrl[0])

        #return ctrlList

