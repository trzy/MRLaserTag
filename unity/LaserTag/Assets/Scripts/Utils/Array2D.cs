public class Array2D<T>
{
    private T[] m_array;
    private int m_lengthDim0;
    private int m_lengthDim1;
    
    public Array2D(T[] array, int lengthDim0, int lengthDim1)
    {
        m_array = array;
        m_lengthDim0 = lengthDim0;
        m_lengthDim1 = lengthDim1;
    }

    public T this[int idx0, int idx1]
    {
        get 
        {
            return m_array[idx0 * m_lengthDim1 + idx1];
        }
        
        set
        {
            m_array[idx0 * m_lengthDim1 + idx1] = value;
        }
    }

    public int Length(int dim)
    {
        if (dim == 0)
        {
            return m_lengthDim0;
        }
        else if (dim == 1)
        {
            return m_lengthDim1;
        }
        return 0;
    }
}
