
using UnityEngine;

/**
* @brief Class representing a single body/link connected to an articulation with a "fixed" joint
*/
public class tntFixedLink : tntChildLink
{	
    public tntFixedLink() : base(0, null)
    {
    }

    public override Vector3 PivotA
    {
        get { return Vector3.zero; }
        set {/*Intentionally empty - this makes no sense for a fixed link*/}
    }

    public override Vector3 PivotB
    {
        get { return Vector3.zero; }
        set { /*Intentionally empty - this makes no sense for a fixed link*/}
    }

    public override bool ArePivotsMatching()
    {
        return true;
    }
    

    /**
     * Fills reducedState state vector with data pertaining to this link
     * @param reducedState state vector to be filled
     * @param offset offset into the vector at which this link is to put its data
     * @remark it's a NOP for tntFixedLink
    */
    public override void FillReducedStateVectorFromCurrentState(tntReducedState reducedState, int offset)
    {}
}
