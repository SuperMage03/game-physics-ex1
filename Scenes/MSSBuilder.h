#pragma once
#include "Renderer.h"
#include "MassSpringSystem.h"

namespace MSSBuilder {

    static MassSpringSystem* createSimpleMSS() {
        MassSpringSystem* MSS = new MassSpringSystem(2, 1);
        MSS->getMassPoint(0)->setPosition(glm::vec3(0.f, 0.f, 0.f));
        MSS->getMassPoint(1)->setPosition(glm::vec3(0.f, 2.f, 0.f));

        MSS->getMassPoint(0)->setVelocity(glm::vec3(-1.f, 0.f, 0.f));
        MSS->getMassPoint(1)->setVelocity(glm::vec3(1.f, 0.f, 0.f));

        MSS->getMassPoint(0)->setMass(10.f);
        MSS->getMassPoint(1)->setMass(10.f);

        MSS->getSpring(0)->setPoint1(0);
        MSS->getSpring(0)->setPoint2(1);
            
        MSS->getSpring(0)->setStiffness(40.f);
        MSS->getSpring(0)->setRestLength(1.f);

        MSS->updateCurrentLengths();

        return MSS;
    }

    static MassSpringSystem* createComplexMSS() {
        MassSpringSystem* MSS = new MassSpringSystem(13, 42);
        // MassSpringSystem* MSS = new MassSpringSystem(13, 78);
        MassPoint* massPoints = MSS->getMassPoints();
        Spring* springs = MSS->getSprings();

        massPoints[0].setPosition(0.f, 0.f, 1.175571f);
        massPoints[1].setPosition(1.051462f, 0.f, 0.5257311f);
        massPoints[2].setPosition(0.3249197f, 1.f, 0.5257311f);
        massPoints[3].setPosition(-0.8506508f, 0.618034f, 0.5257311f);
        massPoints[4].setPosition(-0.8506508f, -0.618034f, 0.5257311f);
        massPoints[5].setPosition(0.3249197f, -1.f, 0.5257311f);
        massPoints[6].setPosition(0.8506508f, 0.618034f, -0.5257311f);
        massPoints[7].setPosition(0.8506508f, -0.618034f, -0.5257311f);
        massPoints[8].setPosition(-0.3249197f, 1.f, -0.5257311f);
        massPoints[9].setPosition(-1.051462f, 0.f, -0.5257311f);
        massPoints[10].setPosition(-0.3249197f, -1.f, -0.5257311f);
        massPoints[11].setPosition(0.f, 0.f, -1.175571f);
        massPoints[12].setPosition(0.f, 0.f, 0.f);

        glm::vec3 shift = glm::vec3(0.f, 0.f, 0.7f);
        float scale = 1.4;

        float mass = 5.f;
        float damping = 2.5f;

        for (int mpId = 0; mpId < 13; mpId++) {
            massPoints[mpId].scalePosition(scale);
            massPoints[mpId].shiftPosition(shift);
            massPoints[mpId].setVelocity(glm::vec3(0.f));
            massPoints[mpId].setMass(mass);
            massPoints[mpId].setDamping(damping);
        }

        springs[0].setPoints(0, 1);
        springs[1].setPoints(0, 2);
        springs[2].setPoints(0, 3);
        springs[3].setPoints(0, 4);
        springs[4].setPoints(0, 5);
        springs[5].setPoints(1, 2);
        springs[6].setPoints(1, 5);
        springs[7].setPoints(1, 6);
        springs[8].setPoints(1, 7);
        springs[9].setPoints(2, 3);
        springs[10].setPoints(2, 6);
        springs[11].setPoints(2, 8);
        springs[12].setPoints(3, 4);
        springs[13].setPoints(3, 8);
        springs[14].setPoints(3, 9);
        springs[15].setPoints(4, 5);
        springs[16].setPoints(4, 9);
        springs[17].setPoints(4, 10);
        springs[18].setPoints(5, 7);
        springs[19].setPoints(5, 10);
        springs[20].setPoints(6, 7);
        springs[21].setPoints(6, 8);
        springs[22].setPoints(6, 11);
        springs[23].setPoints(7, 10);
        springs[24].setPoints(7, 11);
        springs[25].setPoints(8, 9);
        springs[26].setPoints(8, 11);
        springs[27].setPoints(9, 10);
        springs[28].setPoints(9, 11);
        springs[29].setPoints(10, 11);

        springs[30].setPoints(12, 0);
        springs[31].setPoints(12, 1);
        springs[32].setPoints(12, 2);
        springs[33].setPoints(12, 3);
        springs[34].setPoints(12, 4);
        springs[35].setPoints(12, 5);
        springs[36].setPoints(12, 6);
        springs[37].setPoints(12, 7);
        springs[38].setPoints(12, 8);
        springs[39].setPoints(12, 9);
        springs[40].setPoints(12, 10);
        springs[41].setPoints(12, 11);

        float stiffness = 1200.f;
        glm::vec3 edge = massPoints[0].getPosition() - massPoints[1].getPosition();
        float restLength = sqrt(edge.x * edge.x + edge.y * edge.y + edge.z * edge.z);

        for (int sId = 0; sId < 42; sId++) {
            springs[sId].setStiffness(stiffness);
            springs[sId].setRestLength(restLength);
        }

        // float stiffness = 1200.f;

        // int sId = 0;
        // for (int mpId1 = 0; mpId1 < 12; mpId1++) {
        //     for (int mpId2 = mpId1 + 1; mpId2 < 13; mpId2++) {
        //         springs[sId].setPoints(mpId1, mpId2);
        //         springs[sId].setStiffness(stiffness);
        //         glm::vec3 edge = massPoints[mpId1].getPosition() - massPoints[mpId2].getPosition();
        //         float restLength = sqrt(edge.x * edge.x + edge.y * edge.y + edge.z * edge.z);
        //         springs[sId].setRestLength(restLength);
        //         sId++;
        //     }
        // }
        
        MSS->updateCurrentLengths();
        return MSS;
    }
}