--TEAM #21

function sysCall_init() 
    -- Make sure we have version 2.4.13 or above (the particles are not supported otherwise)
    v=sim.getInt32Parameter(sim.intparam_program_version)
    if (v<20413) then
        sim.displayDialog('Warning','The propeller model is only fully supported from CoppeliaSim version 2.4.13 and above.&&nThis simulation will not run as expected!',sim.dlgstyle_ok,false,'',nil,{0.8,0,0,0,0,0})
    end

    -- Detatch the manipulation sphere:
    targetObj=sim.getObjectHandle('Quadricopter_target')
    sim.setObjectParent(targetObj,-1,true)

    -- This control algo was quickly written and is dirty and not optimal. It just serves as a SIMPLE example

    d=sim.getObjectHandle('Quadricopter_base')

    particlesAreVisible=sim.getScriptSimulationParameter(sim.handle_self,'particlesAreVisible')
    sim.setScriptSimulationParameter(sim.handle_tree,'particlesAreVisible',tostring(particlesAreVisible))
    simulateParticles=sim.getScriptSimulationParameter(sim.handle_self,'simulateParticles')
    sim.setScriptSimulationParameter(sim.handle_tree,'simulateParticles',tostring(simulateParticles))

    propellerScripts={-1,-1,-1,-1}
    for i=1,4,1 do
        propellerScripts[i]=sim.getScriptHandle('Quadricopter_propeller_respondable'..i)
    end
    heli=sim.getObjectAssociatedWithScript(sim.handle_self)

    particlesTargetVelocities={0,0,0,0}

    pParam=2
    iParam=0
    dParam=0
    vParam=-2

    cumul=0
    lastE=0
    pAlphaE=0
    pBetaE=0
    psp2=0
    psp1=0

    prevEuler=0

    fakeShadow=sim.getScriptSimulationParameter(sim.handle_self,'fakeShadow')
    if (fakeShadow) then
        shadowCont=sim.addDrawingObject(sim.drawing_discpoints+sim.drawing_cyclic+sim.drawing_25percenttransparency+sim.drawing_50percenttransparency+sim.drawing_itemsizes,0.2,0,-1,1)
    end

    -- Prepare 2 floating views with the camera views:
    floorCam=sim.getObjectHandle('Quadricopter_floorCamera')
    frontCam=sim.getObjectHandle('Quadricopter_frontCamera')
    VSfront = sim.getObjectHandle('VSFront')
    VSleft = sim.getObjectHandle('VSLeft')
    VSright = sim.getObjectHandle('VSRight')
    floorView=sim.floatingViewAdd(0.9,0.9,0.2,0.2,0)
    frontView=sim.floatingViewAdd(0.9,0.7,0.2,0.2,0)
    vsViewFront=sim.floatingViewAdd(0.9,0.5,0.2,0.2,0)
    vsViewLeft=sim.floatingViewAdd(0.9,0.3,0.2,0.2,0)
    vsViewRight=sim.floatingViewAdd(0.9,0.1,0.2,0.2,0)
    sim.adjustView(floorView,floorCam,64)
    sim.adjustView(frontView,frontCam,64)
    sim.adjustView(vsViewFront,VSfront,64)
    sim.adjustView(vsViewLeft,VSleft,64)
    sim.adjustView(vsViewRight,VSright,64)

    max_dist = 2
    
    last_time = sim.getSimulationTime()
    ccw = false
    
    quad = sim.getObjectHandle("lift")
    leftPropeller = sim.getScriptHandle('left_propeller')
    rightPropeller = sim.getScriptHandle('left_propeller#0')
    
    lineC = 0
    lineCleft = 0
    lineCright = 0
    i=1
end

function sysCall_cleanup() 
    sim.removeDrawingObject(shadowCont)
    sim.floatingViewRemove(floorView)
    sim.floatingViewRemove(frontView)
    sim.floatingViewRemove(vsViewFront)
    sim.floatingViewRemove(vsViewLeft)
    sim.floatingViewRemove(vsViewRight)
end 

function sysCall_actuation() 
    lineC = getLINEC(VSfront)
    lineCleft = getLINEC(VSleft)
    lineCright = getLINEC(VSright)
    
    s=sim.getObjectSizeFactor(d)
    
    pos=sim.getObjectPosition(d,-1)
    
    if (fakeShadow) then
        itemData={pos[1],pos[2],0.002,0,0,1,0.2*s}
        sim.addDrawingObjectItem(shadowCont,itemData)
    end
    
    -- Vertical control:
    targetPos=sim.getObjectPosition(targetObj,-1)
    
    
    pos=sim.getObjectPosition(d,-1)
    
    l=sim.getVelocity(heli)
    e=(targetPos[3]-pos[3])
    cumul=cumul+e
    pv=pParam*e
    thrust=5.335+pv+iParam*cumul+dParam*(e-lastE)+l[3]*vParam
    lastE=e
    
    -- Horizontal control: 
    sp=sim.getObjectPosition(targetObj,d)
   --------------------------
    --print(targetPos)
   
    step = 0.01
    
    --0.09 makes HC not move use it as 0
    sim.setScriptSimulationParameter(leftPropeller , 'particleVelocity' , 20)
    sim.setScriptSimulationParameter(rightPropeller , 'particleVelocity' , 20)
    
    if (lineC > 0.1)then
        
        print('wall')
         
        if(i==1) then
            print('first corner')
            for i=1,100 do
                sim.setScriptSimulationParameter(leftPropeller , 'particleVelocity' , 100)
                sim.setScriptSimulationParameter(rightPropeller , 'particleVelocity' , -100)
            end
            for i=1,500 do
            
            end
            i=2
        elseif(i==2) then
            print('second corner')
            for i=1,100 do
                sim.setScriptSimulationParameter(leftPropeller , 'particleVelocity' , -100)
                sim.setScriptSimulationParameter(rightPropeller , 'particleVelocity' , 100)
            end
            for i=1,500 do
            
            end
            i=3
        elseif(i==3) then
            print('third corner')
            for i=1,100 do
                sim.setScriptSimulationParameter(leftPropeller , 'particleVelocity' , 100)
                sim.setScriptSimulationParameter(rightPropeller , 'particleVelocity' , -100)
            end
            for i=1,500 do
            
            end
            i=1
        end
    else
        print('no wall')
        if (lineCleft > 0.1)then
            print('left wall-no front')
            sim.setScriptSimulationParameter(leftPropeller , 'particleVelocity' , 10)
            sim.setScriptSimulationParameter(rightPropeller , 'particleVelocity' , 0.09)
        elseif (lineCright > 0.1)then
            print('right wall-no front')
            sim.setScriptSimulationParameter(leftPropeller , 'particleVelocity' , 0.09)
            sim.setScriptSimulationParameter(rightPropeller , 'particleVelocity' , 10)
        else
            sim.setScriptSimulationParameter(leftPropeller , 'particleVelocity' , 20)
            sim.setScriptSimulationParameter(rightPropeller , 'particleVelocity' , 20)
        end 
    end
   
   --------------------------

    m=sim.getObjectMatrix(d,-1)
    vx={1,0,0}
    vx=sim.multiplyVector(m,vx)
    vy={0,1,0}
    vy=sim.multiplyVector(m,vy)
    alphaE=(vy[3]-m[12])
    alphaCorr=0.25*alphaE+2.1*(alphaE-pAlphaE)
    betaE=(vx[3]-m[12])
    betaCorr=-0.25*betaE-2.1*(betaE-pBetaE)
    pAlphaE=alphaE
    pBetaE=betaE
    alphaCorr=alphaCorr+sp[2]*0.005+1*(sp[2]-psp2)
    betaCorr=betaCorr-sp[1]*0.005-1*(sp[1]-psp1)
    psp2=sp[2]
    psp1=sp[1]
    
    -- Rotational control:
    euler=sim.getObjectOrientation(d,targetObj)
    rotCorr=euler[3]*0.1+2*(euler[3]-prevEuler)
    prevEuler=euler[3]
    
    -- Decide of the motor velocities:
    particlesTargetVelocities[1]=thrust*(1-alphaCorr+betaCorr+rotCorr)
    particlesTargetVelocities[2]=thrust*(1-alphaCorr-betaCorr-rotCorr)
    particlesTargetVelocities[3]=thrust*(1+alphaCorr-betaCorr+rotCorr)
    particlesTargetVelocities[4]=thrust*(1+alphaCorr+betaCorr-rotCorr)
    
    -- Send the desired motor velocities to the 4 rotors:
    for i=1,4,1 do
        sim.setScriptSimulationParameter(propellerScripts[i],'particleVelocity',particlesTargetVelocities[i])
    end
end 

function change(a,i)
    a[i] = a[i] + 0.5
    return a
end

function getLINEC(VS)
    value = sim.getVisionSensorImage(VS + sim.handleflag_greyscale)
    return value[1]
end