wc = rws.getRobWorkStudio():getWorkCell()
state = wc:getDefaultState()
device = wc:findDevice("PA10")
marker = wc:findMovableFrame("Marker");


function setQ(q)
qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6],q[7])
device:setQ(qq,state)
rws.getRobWorkStudio():setState(state)
rw.sleep(0.1)
end

function moveFrame(q)

matR = rw.Rotation3d(q[4],q[5],q[6],q[7],q[8],q[9],q[10],q[11],q[12])
vecP = rw.Vector3d(q[1],q[2],q[3])
trans = rw.Transform3d(vecP,matR)
marker:setTransform(trans,state)
rws.getRobWorkStudio():setState(state)
rw.sleep(0.1)
end

setQ({0,0,-0.819,1.649,0,0,-1.571})

